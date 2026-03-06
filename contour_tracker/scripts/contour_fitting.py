#!/usr/bin/env python3
"""验证轮廓点曲线拟合算法：订阅里程计 → 获取局部轮廓 → 极角排序去重 → B样条拟合 → 实时可视化"""

import json
import math
import threading
import time
import sys
import os
import numpy as np
import pygame
from scipy.interpolate import splprep, splev
from scipy.ndimage import gaussian_filter1d
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
from nng_comm.scripts import Subscriber, Requester, Publisher
from pysfcgal.sfcgal import LineString as SfcgalLineString, Polygon as SfcgalPolygon

# ---------------------------------------------------------------------------
# Pygame 常量
# ---------------------------------------------------------------------------
SCREEN_W, SCREEN_H = 800, 800
BG_COLOR = (30, 30, 30)
AUV_COLOR = (0, 220, 0)
CONTOUR_COLOR = (255, 0, 0)       # 红色轮廓点
CURVE_COLOR = (255, 255, 0)       # 黄色拟合曲线点
OFFSET_COLOR = (0, 255, 0)       # 绿色偏移曲线
TRAJ_COLOR = (0, 200, 255)       # 青色轨迹曲线


# ---------------------------------------------------------------------------
# 工具函数
# ---------------------------------------------------------------------------

def load_jsonc(path: str) -> dict:
    """Load a JSONC file (JSON with // comments, correctly skips // inside strings)"""
    with open(path, 'r', encoding='utf-8') as f:
        content = f.read()
    result, i, in_string = [], 0, False
    while i < len(content):
        c = content[i]
        if c == '\\' and in_string:
            result.append(c); result.append(content[i + 1]); i += 2; continue
        if c == '"':
            in_string = not in_string
        if not in_string and c == '/' and i + 1 < len(content) and content[i + 1] == '/':
            while i < len(content) and content[i] != '\n':
                i += 1
            continue
        result.append(c); i += 1
    return json.loads(''.join(result))


def parse_odometry(msg: dict):
    """解析里程计消息字典，返回 (x, y, heading) 或 None。"""
    try:
        pos = msg["position"]
        ori = msg["orientation"]
        return float(pos["x"]), float(pos["y"]), float(ori["yaw"])
    except (KeyError, TypeError):
        return None


def polar_sort_and_deduplicate(points, cx, cy, threshold_deg, ascending):
    """极角排序 + 去重。

    对每个点计算相对 (cx, cy) 的极角和距离，按极角排序后，
    若相邻两点极角差 < threshold_deg，只保留距离更小的点。
    """
    if len(points) == 0:
        return np.empty((0, 2))

    pts = np.asarray(points, dtype=float)
    dx = pts[:, 0] - cx
    dy = pts[:, 1] - cy
    angles = np.degrees(np.arctan2(dy, dx))
    dists = np.hypot(dx, dy)

    order = np.argsort(angles)
    if not ascending:
        order = order[::-1]

    sorted_angles = angles[order]
    sorted_dists = dists[order]
    sorted_pts = pts[order]

    # 去重：遍历排序后的点，极角差 < threshold 时保留更近的
    keep = [0]
    for i in range(1, len(sorted_pts)):
        angle_diff = abs(sorted_angles[i] - sorted_angles[keep[-1]])
        if angle_diff < threshold_deg:
            # 保留距离更小的
            if sorted_dists[i] < sorted_dists[keep[-1]]:
                keep[-1] = i
        else:
            keep.append(i)

    result_pts = sorted_pts[keep]
    result_angles = sorted_angles[keep]

    # ── 最大间隙旋转：防止 B 样条跨越弧端点产生自交闭环 ──────────
    # 排序后相邻点间最大的角度间隙即为弧的自然断口，
    # 旋转数组使该间隙落在首尾边界，B 样条就不会跨越它。
    if len(result_pts) >= 3:
        diffs = np.abs(np.diff(result_angles))
        wrap_gap = 360.0 - abs(result_angles[-1] - result_angles[0])

        max_interior_idx = int(np.argmax(diffs))
        max_interior_gap = diffs[max_interior_idx]

        if wrap_gap <= max_interior_gap:
            # 最大间隙在内部 → 旋转使间隙后的点成为新起点
            result_pts = np.roll(result_pts, -(max_interior_idx + 1), axis=0)

    return result_pts


def fit_bspline(points, smoothing, num_points):
    """非闭合二次B样条拟合，返回 (curve_x, curve_y, tck, u_new) 或 None。"""
    if len(points) < 3:
        return None
    try:
        x = points[:, 0]
        y = points[:, 1]
        tck, _ = splprep([x, y], k=2, s=smoothing, per=False)
        u_new = np.linspace(0, 1, num_points)
        cx, cy = splev(u_new, tck)
        return cx, cy, tck, u_new
    except Exception as e:
        print(f"[B样条拟合失败] {e}")
        return None


def smooth_ring_kinks(ring_coords, angle_thresh_deg=30, sigma=2.0, window=3):
    """检测闭合环上的尖锐拐点，对其邻域做高斯平滑。"""
    n = len(ring_coords)
    if n < 3:
        return ring_coords

    # 1) 计算每个顶点处的转角（相邻边向量的夹角）
    edges = np.diff(ring_coords, axis=0)                       # (n-1, 2)
    cross = edges[:-1, 0] * edges[1:, 1] - edges[:-1, 1] * edges[1:, 0]
    dot   = np.sum(edges[:-1] * edges[1:], axis=1)
    angles = np.abs(np.arctan2(cross, dot))                    # (n-2,)  对应索引 1..n-2
    thresh = np.radians(angle_thresh_deg)
    kink_mask = np.zeros(n, dtype=bool)
    kink_indices = np.where(angles > thresh)[0] + 1            # 映射回 ring_coords 索引
    for idx in kink_indices:
        lo = max(0, idx - window)
        hi = min(n, idx + window + 1)
        kink_mask[lo:hi] = True

    if not kink_mask.any():
        return ring_coords

    # 2) 对标记区域做高斯平滑
    smoothed = ring_coords.copy()
    sx = gaussian_filter1d(ring_coords[:, 0], sigma=sigma)
    sy = gaussian_filter1d(ring_coords[:, 1], sigma=sigma)
    smoothed[kink_mask, 0] = sx[kink_mask]
    smoothed[kink_mask, 1] = sy[kink_mask]
    return smoothed


def uniformize_by_arclength(points, spacing):
    """沿折线弧长均匀重采样。

    spacing: 期望的点间弧长距离。
    密集处跳过冗余点，稀疏处线性插值补点。
    """
    if len(points) < 2 or spacing <= 0:
        return points
    diffs = np.diff(points, axis=0)
    seg_lens = np.hypot(diffs[:, 0], diffs[:, 1])
    cum_len = np.concatenate(([0.0], np.cumsum(seg_lens)))
    total_len = cum_len[-1]
    if total_len < spacing:
        return points
    targets = np.arange(0, total_len, spacing)
    if targets[-1] < total_len:
        targets = np.append(targets, total_len)
    new_x = np.interp(targets, cum_len, points[:, 0])
    new_y = np.interp(targets, cum_len, points[:, 1])
    return np.column_stack((new_x, new_y))


def detect_collision_ahead(contour_pts, px, py, heading,
                            sector_range=40.0, sector_half_angle_deg=12.0):
    # debug 35, 12
    """检测 AUV 前方扇形区域内是否存在轮廓点。

    返回 True 表示存在碰撞风险。
    sector_range: 扇形最大距离 (m)
    sector_half_angle_deg: 扇形半角 (°)
    """
    if contour_pts is None or len(contour_pts) == 0:
        return False
    half_angle = math.radians(sector_half_angle_deg)
    for pt in contour_pts:
        dx, dy = pt[0] - px, pt[1] - py
        d = math.sqrt(dx * dx + dy * dy)
        if d > sector_range or d < 1e-6:
            continue
        phi = math.atan2(dy, dx)
        delta = (phi - heading + math.pi) % (2 * math.pi) - math.pi
        if abs(delta) <= half_angle:
            return True
    return False


def generate_arc_trajectory(px, py, heading, r_min, num_points,
                             turn_left=True, spacing=2.0):
    """生成圆弧规避轨迹。

    turn_left=True → 左转（R 模式）；False → 右转（L 模式）。
    spacing: 相邻点的弧长间距。
    """
    if turn_left:
        cx = px + r_min * math.cos(heading - math.pi / 2)
        cy = py + r_min * math.sin(heading - math.pi / 2)
        sign = -1.0  # CW in math coords = left turn on screen (y-down)
    else:
        cx = px + r_min * math.cos(heading + math.pi / 2)
        cy = py + r_min * math.sin(heading + math.pi / 2)
        sign = 1.0   # CCW in math coords = right turn on screen (y-down)

    alpha0 = math.atan2(py - cy, px - cx)
    d_alpha = sign * spacing / r_min

    pts = []
    for i in range(num_points):
        a = alpha0 + (i + 1) * d_alpha
        pts.append([cx + r_min * math.cos(a),
                     cy + r_min * math.sin(a)])
    return np.array(pts)


def compute_offset_curve(curve_x, curve_y, d_ref, auv_x, auv_y, disk_n=32,
                         kink_angle=30, kink_sigma=2.0, kink_window=3,
                         resample_spacing=0):
    """通过 Minkowski Sum（线段 ⊕ 圆盘）计算 AUV 侧偏移曲线。返回 np.ndarray (N,2)。

    整体思路：
      想象拿一支半径为 d_ref 的圆形印章，沿着曲线从头滑到尾，
      印章扫过的区域就是 Minkowski Sum 的结果——一个"腊肠形"多边形。
      这个腊肠的外轮廓天然就是曲线向两侧各偏移 d_ref 后的包络，
      再加上两端的半圆帽。我们只需要从中取出 AUV 那一侧的半边即可。

      曲线           ──────────────
                    ╱                ╲
      AUV 侧弧  →  │  腊肠形多边形   │  ← 背侧弧
                    ╲                ╱
                     ──────────────
                   ↑端帽            ↑端帽
    """

    # ── 第 1 步：构造 SFCGAL 线段对象 ──────────────────────────────
    # 把拟合曲线的采样点 (curve_x, curve_y) 转为 SFCGAL 的 LineString。
    # 注意：SFCGAL 不允许相邻两个点完全重合（距离为 0），否则会报错，
    # 所以这里遍历时跳过与前一个点坐标完全相同的点。
    coords = []
    for x, y in zip(curve_x, curve_y):
        if not coords or (x != coords[-1][0] or y != coords[-1][1]):
            coords.append((float(x), float(y)))
    # 如果去重后不足两个点，无法构成线段，直接返回原曲线
    if len(coords) < 2:
        return np.column_stack([curve_x, curve_y])
    line = SfcgalLineString(coords)

    # ── 第 2 步：构造圆盘（正多边形逼近） ─────────────────────────
    # 真正的圆盘无法精确表示，用正 disk_n 边形来近似。
    # disk_n=32 意味着用正 32 边形，已经非常接近圆。
    # 圆盘中心在原点 (0,0)，半径为 d_ref。
    # 在 0~2π 上均匀取 disk_n 个角度，算出每个顶点坐标。
    angles = np.linspace(0, 2 * np.pi, disk_n, endpoint=False)
    disk_coords = [(float(d_ref * np.cos(a)), float(d_ref * np.sin(a))) for a in angles]
    disk_coords.append(disk_coords[0])  # 首尾相连，形成闭合多边形
    disk = SfcgalPolygon(disk_coords)

    # ── 第 3 步：执行 Minkowski Sum ───────────────────────────────
    # Minkowski Sum (闵可夫斯基和) 的直观理解：
    #   把圆盘的圆心放在曲线上的每一个点，圆盘扫过的所有面积的并集。
    #   结果是一个"腊肠形"多边形——曲线两侧各膨胀了 d_ref，
    #   两端各多出一个半圆形的"帽子"。
    sausage = line.minkowski_sum(disk)

    # ── 第 4 步：提取外边界环的坐标 ───────────────────────────────
    # Minkowski Sum 的结果通常是一个 Polygon（单个多边形）。
    # 极端情况下（如曲线自交）可能产生 MultiPolygon（多个多边形），
    # 此时取面积最大的那个，因为它包含主体轮廓。
    # 我们只需要它的外边界环 (exterior ring)，即最外面那一圈的顶点序列。
    geom_type = sausage.geom_type
    if geom_type == "Polygon":
        ring = sausage.exterior
    elif geom_type == "MultiPolygon":
        largest = max(sausage.geoms, key=lambda g: g.area)
        ring = largest.exterior
    else:
        return np.column_stack([curve_x, curve_y])

    # ring 是一个闭合环（首尾点相同），转成 numpy 数组方便后续计算
    ring_coords = np.array(ring.to_coordinates())

    # ── 第 4.5 步：平滑闭合环上的尖锐拐点 ──────────────────────────
    # Minkowski Sum 产生的环在折线顶点处会出现尖锐拐角（两段偏移弧交汇处），
    # 对这些拐点的邻域做局部高斯平滑，消除方向突变，保持整体形状。
    ring_coords = smooth_ring_kinks(ring_coords, kink_angle, kink_sigma, kink_window)

    if resample_spacing > 0:
        ring_coords = uniformize_by_arclength(ring_coords, resample_spacing)

    # [DEBUG] 直接返回完整闭合环，跳过后续弧段提取，用于可视化调试
    return ring_coords


# ---------------------------------------------------------------------------
# 里程计订阅线程
# ---------------------------------------------------------------------------

class OdometrySubscriber:
    """后台订阅里程计数据，更新共享位姿。"""

    def __init__(self, address):
        self.address = address
        self.lock = threading.Lock()
        self._x = 0.0
        self._y = 0.0
        self._heading = 0.0
        self._has_data = False
        self._sub = Subscriber()

    def _on_message(self, msg: dict):
        result = parse_odometry(msg)
        if result is not None:
            with self.lock:
                self._x, self._y, self._heading = result
                self._has_data = True

    def start(self):
        if not self._sub.init(self.address):
            print(f"[里程计] 初始化失败: {self.address}")
            return
        print(f"[里程计] 正在连接 {self.address} ...")
        self._sub.set_callback(self._on_message)
        self._sub.start_async()

    def stop(self):
        self._sub.close()

    def get_pose(self):
        """返回 (x, y, heading, has_data)。"""
        with self.lock:
            return self._x, self._y, self._heading, self._has_data


# ---------------------------------------------------------------------------
# 主函数
# ---------------------------------------------------------------------------

def world_to_screen(wx, wy, cam_x, cam_y, zoom):
    """Convert world coordinates to screen pixel coordinates (Y-flipped)."""
    sx = int((wx - cam_x) * zoom + SCREEN_W / 2)
    sy = int((wy - cam_y) * zoom + SCREEN_H / 2)
    return sx, sy


def draw_auv(screen, sx, sy, heading):
    """Draw AUV as a green triangle pointing in heading direction."""
    size = 20
    angles = [heading, heading + 2.5, heading - 2.5]
    lengths = [size, size * 0.6, size * 0.6]
    points = []
    for a, l in zip(angles, lengths):
        px = sx + l * math.cos(a)
        py = sy + l * math.sin(a)
        points.append((int(px), int(py)))
    pygame.draw.polygon(screen, (255, 255, 255), points)
    pygame.draw.polygon(screen, AUV_COLOR, points, 2)


def main():
    # 加载配置
    config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "..", "config", "contour_fitting.jsonc")
    cfg = load_jsonc(config_path)

    odom_addr = cfg["odometry_address"]
    contour_addr = cfg["contour_service_address"]
    contour_r = cfg["contour_radius"]
    query_hz = cfg["query_hz"]
    angle_thresh = cfg["angle_threshold_deg"]
    ascending = cfg["sort_ascending"]
    smoothing = cfg["bspline_smoothing"]
    num_pts = cfg["bspline_num_points"]
    d_ref = cfg["d_ref"]
    resample_ds = cfg.get("resample_spacing", 0)
    # 拐点平滑参数：
    # kink_angle_thresh_deg — 转角阈值（度），相邻边夹角超过此值的顶点视为拐点
    # kink_smooth_sigma     — 高斯滤波标准差，越大平滑范围越广、过渡越柔和
    # kink_smooth_window    — 拐点向两侧扩展的点数，决定每个拐点的平滑邻域大小
    kink_angle = cfg.get("kink_angle_thresh_deg", 30)
    kink_sigma = cfg.get("kink_smooth_sigma", 2.0)
    kink_window = cfg.get("kink_smooth_window", 3)
    traj_num = cfg.get("trajectory_num_points", 20)
    tracking_mode = cfg.get("tracking_mode", "R")
    min_turn_radius = cfg.get("min_turn_radius", 10)
    collision_weight = cfg.get("collision_avoidance_weight", 0.7)
    follow_auv = cfg.get("follow_auv", False)
    traj_pub_addr = cfg.get("trajectory_publish_address", "")
    closure_endpoint_cfg = cfg.get("closure_endpoint", None)
    query_interval = 1.0 / query_hz

    # 启动里程计订阅
    odom_sub = OdometrySubscriber(odom_addr)
    odom_sub.start()

    # 轮廓服务 Req socket
    req = Requester()
    req.init(contour_addr)
    print(f"[轮廓服务] 正在连接 {contour_addr} ...")

    # 轨迹发布 Pub socket
    traj_pub = None
    if traj_pub_addr:
        traj_pub = Publisher()
        traj_pub.init(traj_pub_addr)
        print(f"[轨迹发布] 正在监听 {traj_pub_addr} ...")

    # Pygame 初始化
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_W, SCREEN_H))
    pygame.display.set_caption("Contour B-Spline Fitting")
    clock = pygame.time.Clock()

    # 相机状态
    cam_x, cam_y = 0.0, 0.0
    zoom = 10.0
    min_zoom, max_zoom = 1.0, 200.0
    dragging = False
    drag_start = (0, 0)
    cam_start = (0.0, 0.0)

    # 数据查询节流
    last_query_time = 0.0
    sorted_pts = None
    curve = None
    offset_pts = None
    prev_trajectory_pts = None
    trajectory_pts = None
    last_published_pts = None

    # 闭合检测状态
    closure_origin = None          # AUV 初始位置 (x, y)
    closure_target = None          # 闭合检测目标点 (x, y)，可能是初始位置或配置的终点
    closure_departed = False       # 是否已离开初始位置 50m
    closure_detected = False       # 是否已检测到闭合（回到初始点 20m 内）
    closure_empty_sent = 0         # 已发送空轨迹的次数
    CLOSURE_DEPART_DIST = 50.0     # 离开阈值 (m)
    CLOSURE_RETURN_DIST = 20.0     # 回到阈值 (m)
    CLOSURE_EMPTY_COUNT = 3        # 闭合后发送空轨迹的次数

    if closure_endpoint_cfg is not None:
        closure_target = tuple(closure_endpoint_cfg)
        print(f"[闭合检测] 使用配置闭合终点: ({closure_target[0]:.1f}, {closure_target[1]:.1f})")

    print("[主循环] 启动，等待里程计数据 ...")

    running = True
    try:
        while running:
            # --- 事件处理 ---
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1 and not follow_auv:
                        dragging = True
                        drag_start = event.pos
                        cam_start = (cam_x, cam_y)
                    elif event.button == 4:  # scroll up
                        zoom = min(max_zoom, zoom * 1.15)
                    elif event.button == 5:  # scroll down
                        zoom = max(min_zoom, zoom / 1.15)
                elif event.type == pygame.MOUSEBUTTONUP:
                    if event.button == 1:
                        dragging = False
                elif event.type == pygame.MOUSEMOTION:
                    if dragging and not follow_auv:
                        dx = event.pos[0] - drag_start[0]
                        dy = event.pos[1] - drag_start[1]
                        cam_x = cam_start[0] - dx / zoom
                        cam_y = cam_start[1] - dy / zoom

            if not running:
                break

            # --- 数据获取（按 query_hz 节流） ---
            px, py, heading, has_data = odom_sub.get_pose()
            if follow_auv and has_data:
                cam_x = px
                cam_y = py

            # --- 闭合检测 ---
            if has_data and not closure_detected:
                if closure_origin is None:
                    closure_origin = (px, py)
                    print(f"[闭合检测] 记录初始位置: ({px:.1f}, {py:.1f})")
                    # 若未配置闭合终点，则使用初始位置
                    if closure_target is None:
                        closure_target = closure_origin
                        print(f"[闭合检测] 未配置闭合终点，使用初始位置作为闭合点")
                else:
                    dist_to_origin = math.hypot(px - closure_origin[0], py - closure_origin[1])
                    if not closure_departed:
                        if dist_to_origin >= CLOSURE_DEPART_DIST:
                            closure_departed = True
                            print(f"[闭合检测] 已离开初始位置 {dist_to_origin:.1f}m，开始检测闭合")
                    else:
                        dist_to_target = math.hypot(px - closure_target[0], py - closure_target[1])
                        if dist_to_target <= CLOSURE_RETURN_DIST:
                            closure_detected = True
                            print(f"[闭合检测] 闭合完成！距闭合点 {dist_to_target:.1f}m，停止发布轨迹")

            now = time.time()
            if has_data and now - last_query_time >= query_interval:
                last_query_time = now

                # 调用轮廓服务
                contour_points = None
                try:
                    data = req.request({"x": px, "y": py, "r": contour_r}, timeout_ms=3000)
                    if data is not None:
                        raw_pts = data.get("points", [])
                        if raw_pts:
                            contour_points = np.array(raw_pts, dtype=float)
                    else:
                        print("[轮廓服务] 请求超时")
                except Exception as e:
                    print(f"[轮廓服务] 异常: {e}")
                    req.close()
                    req = Requester()
                    req.init(contour_addr)

                # 极角排序 + 去重
                sorted_pts = None
                if contour_points is not None and len(contour_points) > 0:
                    sorted_pts = polar_sort_and_deduplicate(
                        contour_points, px, py, angle_thresh, ascending
                    )

                # B样条拟合
                curve = None
                offset_pts = None
                if sorted_pts is not None and len(sorted_pts) >= 3:
                    curve = fit_bspline(sorted_pts, smoothing, num_pts)
                if curve is not None:
                    cx, cy, tck, u_new = curve
                    offset_pts = compute_offset_curve(
                        cx, cy, d_ref, px, py,
                        kink_angle=kink_angle,
                        kink_sigma=kink_sigma,
                        kink_window=kink_window,
                        resample_spacing=resample_ds,
                    )

                # 计算 offset_pts 中位于 AUV 正左侧（航向左侧法线）±10°、40 m 范围内的点索引
                front_indices = set()
                if offset_pts is not None and len(offset_pts) > 0 and has_data:
                    max_range = 50.0 # debug 40.0
                    left_normal = heading - math.pi / 2  # 左侧法线方向
                    rel = offset_pts - np.array([px, py])
                    dists = np.linalg.norm(rel, axis=1)
                    angles = np.arctan2(rel[:, 1], rel[:, 0])
                    diff = (angles - left_normal + math.pi) % (2 * math.pi) - math.pi
                    candidates = set(np.where((dists <= max_range) & (np.abs(diff) <= math.radians(10)))[0])
                    # 仅保留距 AUV 最近点之后至少 20 个点的候选，且至多60个点
                    nearest_idx = int(np.argmin(dists))
                    n = len(offset_pts)
                    after_set = set((nearest_idx + k) % n for k in range(20, min(60, n)))
                    front_indices = candidates & after_set

                trajectory_pts = None
                if offset_pts is not None and len(offset_pts) > 0:
                    dists = np.linalg.norm(offset_pts - np.array([px, py]), axis=1)
                    nearest_idx = int(np.argmin(dists))
                    tail = offset_pts[nearest_idx:]
                    if len(tail) < traj_num:
                        # 从开头补充，但最多补到 nearest_idx 前一个点
                        max_wrap = min(traj_num - len(tail), nearest_idx)
                        tail = np.concatenate([tail, offset_pts[:max_wrap]])
                    else:
                        tail = tail[:traj_num]
                    new_traj = tail
                    if len(new_traj) >= 2:
                        if (prev_trajectory_pts is not None
                                and len(prev_trajectory_pts) == len(new_traj)):
                            alpha = 0.3  # 新值权重，越小越平滑
                            trajectory_pts = alpha * new_traj + (1 - alpha) * prev_trajectory_pts
                        else:
                            trajectory_pts = new_traj
                        prev_trajectory_pts = trajectory_pts.copy()

                # ── 碰撞检测 & 规避轨迹混合 ──
                if (trajectory_pts is not None
                        and sorted_pts is not None
                        and (detect_collision_ahead(sorted_pts, px, py, heading)
                             or len(front_indices) > 0)):
                    turn_left = (tracking_mode == "R")
                    evasion_traj = generate_arc_trajectory(
                        px, py, heading, min_turn_radius,
                        len(trajectory_pts), turn_left=turn_left,
                        spacing=resample_ds if resample_ds > 0 else 2.0)
                    # 触发规避时直接用规避轨迹取代原来的轨迹
                    trajectory_pts = evasion_traj
                    # trajectory_pts = (collision_weight * evasion_traj
                    #                   + (1 - collision_weight) * trajectory_pts)

            # --- 发布轨迹（闭合后发送空轨迹再停止） ---
            if closure_detected:
                trajectory_pts = None
                if traj_pub is not None and closure_empty_sent < CLOSURE_EMPTY_COUNT:
                    msg = {
                        "timestamp": time.time(),
                        "points": [],
                    }
                    traj_pub.publish(msg)
                    closure_empty_sent += 1
                    print(f"[闭合检测] 发送空轨迹 ({closure_empty_sent}/{CLOSURE_EMPTY_COUNT})")
            if traj_pub is not None and trajectory_pts is not None:
                if last_published_pts is None or not np.array_equal(trajectory_pts, last_published_pts):
                    msg = {
                        "timestamp": time.time(),
                        "points": trajectory_pts.tolist(),
                    }
                    traj_pub.publish(msg)
                    last_published_pts = trajectory_pts.copy()

            # --- 绘制 ---
            screen.fill(BG_COLOR)

            # 原点十字线
            ox, oy = world_to_screen(0.0, 0.0, cam_x, cam_y, zoom)
            pygame.draw.line(screen, (100, 100, 100), (ox - 10, oy), (ox + 10, oy), 1)
            pygame.draw.line(screen, (100, 100, 100), (ox, oy - 10), (ox, oy + 10), 1)

            # AUV 三角形
            if has_data:
                asx, asy = world_to_screen(px, py, cam_x, cam_y, zoom)
                draw_auv(screen, asx, asy, heading)

            # 轮廓点（红色圆点）
            if sorted_pts is not None and len(sorted_pts) > 0:
                for pt in sorted_pts:
                    sx, sy = world_to_screen(pt[0], pt[1], cam_x, cam_y, zoom)
                    pygame.draw.circle(screen, CONTOUR_COLOR, (sx, sy), 3)

            # 拟合曲线（黄色圆点）
            if curve is not None:
                cx, cy, tck, u_new = curve
                for i in range(len(cx)):
                    sx, sy = world_to_screen(cx[i], cy[i], cam_x, cam_y, zoom)
                    pygame.draw.circle(screen, CURVE_COLOR, (sx, sy), 2)

            # 偏移曲线（绿色圆点，前方锥形区域内的点为紫色）
            if offset_pts is not None and len(offset_pts) > 0:
                for i, pt in enumerate(offset_pts):
                    sx, sy = world_to_screen(pt[0], pt[1], cam_x, cam_y, zoom)
                    color = (180, 0, 255) if i in front_indices else OFFSET_COLOR
                    pygame.draw.circle(screen, color, (sx, sy), 2)

            # 轨迹曲线（青色带箭头）
            if trajectory_pts is not None and len(trajectory_pts) >= 2:
                scr = [world_to_screen(pt[0], pt[1], cam_x, cam_y, zoom)
                       for pt in trajectory_pts]
                pygame.draw.lines(screen, TRAJ_COLOR, False, scr, 3)
                # 每隔 3 段绘制一个箭头
                for i in range(0, len(scr) - 1, 3):
                    x0, y0 = scr[i]
                    x1, y1 = scr[i + 1]
                    mx, my = (x0 + x1) / 2, (y0 + y1) / 2
                    angle = math.atan2(y1 - y0, x1 - x0)
                    arrow_len = 18
                    left  = (mx - arrow_len * math.cos(angle - 0.4),
                             my - arrow_len * math.sin(angle - 0.4))
                    right = (mx - arrow_len * math.cos(angle + 0.4),
                             my - arrow_len * math.sin(angle + 0.4))
                    pygame.draw.polygon(screen, TRAJ_COLOR,
                                        [(int(mx), int(my)),
                                         (int(left[0]), int(left[1])),
                                         (int(right[0]), int(right[1]))])

            # 标题栏显示 AUV 坐标和航向
            if has_data:
                pygame.display.set_caption(
                    f"AUV ({px:.1f}, {py:.1f})  heading={math.degrees(heading):.1f}°"
                )

            pygame.display.flip()
            clock.tick(30)

    except KeyboardInterrupt:
        print("\n[退出] Ctrl+C")
    finally:
        pygame.quit()
        req.close()
        if traj_pub is not None:
            traj_pub.close()
        odom_sub.stop()


if __name__ == "__main__":
    main()
