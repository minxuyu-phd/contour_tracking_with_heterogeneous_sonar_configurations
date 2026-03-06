# OGM2D — 二维占据栅格地图

基于四叉树稀疏存储与贝叶斯对数几率融合的二维占据栅格地图系统。系统通过 pynng 订阅里程计（Odometry）、机械扫描成像声呐（MSIS）和左右两路单波束测深仪（SBES Left / SBES Right）四路传感器数据，在线更新栅格概率，并提供轮廓提取服务和 PyGame 实时可视化。

---

## 文件结构

```
MxyOGM2D/
├── main.py              # 入口：参数解析、组件装配、启动/停止
├── config.json          # 默认配置文件
└── core/
    ├── __init__.py      # 包声明
    ├── config.py        # 配置加载与校验（dataclass）
    ├── sensors.py       # 传感器数据结构与解析
    ├── transforms.py    # 坐标变换 & 波束足迹采样
    ├── quadtree.py      # 动态四叉树（稀疏对数几率存储）
    ├── grid.py          # OccupancyGrid：贝叶斯更新 + 查询
    ├── contour.py       # 轮廓提取（边界占据单元）
    ├── colormap.py      # Jet 色表 LUT（概率→颜色）
    ├── comm.py          # pynng 通信：Sub0 订阅 + Rep0 服务
    └── visualizer.py    # PyGame 可视化（拖拽/缩放/AUV/轮廓）
```

---

## 环境与启动

### 依赖

- Python ≥ 3.10
- numpy
- pynng
- pygame（可选，仅可视化需要）

### 运行

```bash
# 默认配置启动
python main.py

# 指定配置文件
python main.py --config path/to/config.json

# 禁用可视化（纯后台模式）
python main.py --no-viz

# 调试日志
python main.py -v
```

### CLI 参数

| 参数 | 说明 |
|------|------|
| `--config` | JSON 配置文件路径，默认 `config.json` |
| `--no-viz` | 禁用 PyGame 可视化 |
| `--verbose`, `-v` | 启用 DEBUG 级别日志 |

---

## 并发模型

系统共 6 个线程：

| 线程名 | 类型 | 协议 | 职责 |
|--------|------|------|------|
| `sub-odometry` | `SensorSubscriber` (daemon) | Sub0 | 订阅里程计，更新 AUV 位姿 |
| `sub-msis` | `SensorSubscriber` (daemon) | Sub0 | 订阅 MSIS 数据，更新栅格 |
| `sub-sbes-left` | `SensorSubscriber` (daemon) | Sub0 | 订阅左舷 SBES 数据，更新栅格 |
| `sub-sbes-right` | `SensorSubscriber` (daemon) | Sub0 | 订阅右舷 SBES 数据，更新栅格 |
| `rep-contour` | `ContourServer` (daemon) | Rep0 | 接收圆形区域查询，返回轮廓点 |
| `MainThread` | — | — | PyGame 事件循环 / 空闲等待 |

### 停止协议

1. 收到 `SIGINT`（Ctrl+C）或 PyGame 窗口关闭 / ESC 键
2. 设置 `threading.Event` (`stop_event`)
3. 各订阅线程在 `recv_timeout=500ms` 后检测到事件并退出
4. 主线程对每个子线程调用 `join(timeout=2.0)`

---

## 数据流

```
 ┌────────────┐  ┌────────────┐  ┌────────────┐  ┌────────────┐
 │ Odometry   │  │   MSIS     │  │ SBES Left  │  │ SBES Right │
 │ Publisher  │  │ Publisher  │  │ Publisher  │  │ Publisher  │
 └─────┬──────┘  └─────┬──────┘  └─────┬──────┘  └─────┬──────┘
       │ nng/sub       │ nng/sub       │ nng/sub       │ nng/sub
       ▼               ▼               ▼               ▼
 ┌───────────┐  ┌───────────┐  ┌─────────────┐  ┌──────────────┐
 │sub-odometry│ │ sub-msis  │  │sub-sbes-left│  │sub-sbes-right│
 │ parse →    │ │ parse →   │  │ parse →     │  │ parse →      │
 │ update_pose│ │update_msis│  │ update_sbes │  │ update_sbes  │
 └─────┬──────┘ └─────┬─────┘  └──────┬──────┘  └──────┬───────┘
       │               │               │                │
       ▼               ▼               ▼                ▼
 ┌─────────────────────────────────────────────┐
 │              OccupancyGrid                  │
 │  pose (RLock)  │  QuadTree (RLock)          │
 │                │  log-odds per cell         │
 └──────────┬──────────────┬───────────────────┘
            │              │
            ▼              ▼
 ┌──────────────┐   ┌──────────────────┐
 │  Visualizer  │   │  ContourServer   │
 │  (PyGame)    │   │  (Rep0)          │
 │  query_rect  │   │  extract_contour │
 └──────────────┘   └──────────────────┘
                           ▲
                           │ nng/req-rep
                    ┌──────┴───────┐
                    │ 外部客户端    │
                    │ JSON 请求/响应│
                    └──────────────┘
```

---

## 配置文件

`config.json` 模式：

```jsonc
{
  "cell_size": 0.2,                // 栅格单元边长（米）
  "sensors": {
    "odometry": {
      "address": "tcp://127.0.0.1:5001"   // nng Sub0 地址（每个传感器独立地址，无话题前缀）
    },
    "msis": {
      "address": "tcp://127.0.0.1:5002",
      "confidence": 0.9,          // MSIS 贝叶斯置信度 (0, 1)
      "aperture": 2.0,            // 水平波束展宽角（度），用于高斯扩展命中格
      "bin_threshold": 120         // MSIS 绝对 bin 强度阈值 (uint8)，低于此值的回波被忽略
    },
    "sbes_left": {
      "address": "tcp://127.0.0.1:5003",
      "confidence": 0.60,         // SBES 贝叶斯置信度 (0, 1)，每路可独立设置
      "beam_angle": -75.0,        // 传感器安装角（度），相对体坐标系（正值=顺时针/右舷）
      "aperture": 25.0            // 水平波束展宽角（度），用于高斯扩展命中格
    },
    "sbes_right": {
      "address": "tcp://127.0.0.1:5004",
      "confidence": 0.60,
      "beam_angle": 75.0,
      "aperture": 25.0
    }
  },
  "service": {
    "address": "tcp://0.0.0.0:5010",      // ContourServer 监听地址
    "contour_threshold": 0.65              // 轮廓提取占据概率阈值
  },
  "visualization": {
    "enabled": true,              // 是否启用 PyGame 窗口
    "window_width": 1024,         // 窗口宽度（像素）
    "window_height": 768,         // 窗口高度（像素）
    "fps": 30,                    // 帧率上限
    "camera_x": 120.0,            // 初始摄像机中心 X（世界坐标）
    "camera_y": 0.0               // 初始摄像机中心 Y（世界坐标）
  }
}
```

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `cell_size` | float | 0.1 | 栅格分辨率（米/格） |
| `sensors.*.address` | str | — | pynng 传感器端点（每个传感器独立地址，无话题前缀） |
| `sensors.msis.confidence` | float | 0.9 | MSIS 贝叶斯更新置信度 |
| `sensors.msis.aperture` | float | 0.0 | MSIS 波束展宽角（度），用于高斯扩展命中格 |
| `sensors.msis.bin_threshold` | int | 0 | MSIS 绝对 bin 强度阈值（uint8），低于此值的回波被忽略；0 = 接受全部 |
| `sensors.sbes_*.confidence` | float | 0.9 | SBES 贝叶斯更新置信度（每路可独立设置） |
| `sensors.sbes_*.beam_angle` | float | 0.0 | SBES 安装角（度），相对体坐标系（正值=顺时针/右舷） |
| `sensors.sbes_*.aperture` | float | 0.0 | SBES 波束展宽角（度），用于高斯扩展命中格；0 = 不展宽 |
| `service.address` | str | `tcp://0.0.0.0:5010` | 轮廓服务绑定地址 |
| `service.contour_threshold` | float | 0.65 | 占据概率 ≥ 此值视为"已占据" |
| `visualization.enabled` | bool | true | 启用可视化 |
| `visualization.window_width` | int | 1024 | 窗口宽度 |
| `visualization.window_height` | int | 768 | 窗口高度 |
| `visualization.fps` | int | 30 | 帧率上限 |
| `visualization.camera_x` | float | 120.0 | 初始摄像机中心 X（世界坐标） |
| `visualization.camera_y` | float | 0.0 | 初始摄像机中心 Y（世界坐标） |

---

## 模块 API 参考

### `main.py` — 入口

程序入口，负责参数解析、配置加载、组件创建、线程启动与关停。

**函数**

- `main() -> None` — 解析 CLI 参数，加载配置，创建 `OccupancyGrid`，启动订阅线程和轮廓服务，运行可视化或空闲等待，处理 `SIGINT` 优雅退出。

---

### `ogm/config.py` — 配置加载与校验

将 JSON 配置文件映射为类型安全的 dataclass 层次结构。

**类**

#### `SensorConfig`

传感器连接配置。

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `address` | `str` | — | pynng 端点地址 |
| `confidence` | `float` | `0.9` | 贝叶斯更新置信度 |
| `beam_angle` | `float` | `0.0` | 安装角（度），相对体坐标系 |
| `aperture` | `float` | `0.0` | 波束展宽角（度），0 = 不展宽 |
| `bin_threshold` | `int` | `0` | MSIS 绝对 bin 阈值（uint8），0 = 接受全部 |

方法：
- `from_dict(cls, d: dict[str, Any]) -> SensorConfig` — 从字典构造。

#### `ServiceConfig`

轮廓服务配置。

| 字段 | 类型 | 默认值 |
|------|------|--------|
| `address` | `str` | `"tcp://0.0.0.0:5010"` |
| `contour_threshold` | `float` | `0.65` |

方法：
- `from_dict(cls, d: dict[str, Any]) -> ServiceConfig`

#### `VisualizationConfig`

可视化配置。

| 字段 | 类型 | 默认值 |
|------|------|--------|
| `enabled` | `bool` | `True` |
| `window_width` | `int` | `1024` |
| `window_height` | `int` | `768` |
| `fps` | `int` | `30` |
| `camera_x` | `float` | `120.0` |
| `camera_y` | `float` | `0.0` |

方法：
- `from_dict(cls, d: dict[str, Any]) -> VisualizationConfig`

#### `AppConfig`

顶层应用配置，聚合所有子配置。

| 字段 | 类型 | 默认值 |
|------|------|--------|
| `cell_size` | `float` | `0.1` |
| `odometry` | `SensorConfig` | `SensorConfig("tcp://127.0.0.1:5001")` |
| `msis` | `SensorConfig` | `SensorConfig("tcp://127.0.0.1:5002")` |
| `sbes_left` | `SensorConfig` | `SensorConfig("tcp://127.0.0.1:5003", 0.85)` |
| `sbes_right` | `SensorConfig` | `SensorConfig("tcp://127.0.0.1:5004", 0.85)` |
| `service` | `ServiceConfig` | `ServiceConfig()` |
| `visualization` | `VisualizationConfig` | `VisualizationConfig()` |

方法：
- `from_dict(cls, d: dict[str, Any]) -> AppConfig` — 从字典构造。
- `load(cls, path: str | Path) -> AppConfig` — 从 JSON 文件加载。

---

### `ogm/sensors.py` — 传感器数据结构与解析

定义传感器数据载体（dataclass）、原始读数中间类型和 JSON 解析函数。

**类**

#### `OdometryData`

AUV 位姿数据。

| 字段 | 类型 | 说明 |
|------|------|------|
| `timestamp` | `float` | 时间戳 |
| `x` | `float` | 世界 X（米） |
| `y` | `float` | 世界 Y（米） |
| `heading` | `float` | 航向角（弧度，0=东，逆时针正） |

#### `MSISData`

机械扫描成像声呐数据。

| 字段 | 类型 | 说明 |
|------|------|------|
| `timestamp` | `float` | 时间戳 |
| `x` | `float` | 声呐世界 X |
| `y` | `float` | 声呐世界 Y |
| `heading` | `float` | AUV 航向角 |
| `beam_angle` | `float` | 波束方向（体坐标系，弧度） |
| `range_max` | `float` | 最大量程（米） |
| `bin_values` | `np.ndarray` | 强度分仓，shape `(N,)` |

#### `SBESData`

单波束测深仪数据。

| 字段 | 类型 | 说明 |
|------|------|------|
| `timestamp` | `float` | 时间戳 |
| `x` | `float` | 换能器世界 X |
| `y` | `float` | 换能器世界 Y |
| `heading` | `float` | 波束方向（世界坐标系，弧度） |
| `range_m` | `float` | 测量距离（米） |

#### `RawSBESReading`

原始 SBES 读数（不含位置信息，由 handler 补齐）。

| 字段 | 类型 | 说明 |
|------|------|------|
| `distance` | `float` | 测量距离（米） |
| `detected` | `bool` | 是否检测到目标 |

#### `RawMSISReading`

原始 MSIS 波束读数（不含位置信息，由 handler 补齐）。

| 字段 | 类型 | 说明 |
|------|------|------|
| `beam_angle` | `float` | 波束方向（体坐标系，弧度） |
| `range_max` | `float` | 最大量程（米） |
| `bin_values` | `np.ndarray` | 强度分仓，uint8 |

**函数**

- `parse_odometry(payload: bytes) -> OdometryData | None` — 解析里程计 JSON。提取 `position.x/y`、`orientation.yaw`、`timestamp`。
- `parse_sbes(payload: bytes) -> RawSBESReading | None` — 解析 SBES JSON。`detected=false` 时返回 `None`。
- `parse_msis(payload: bytes) -> RawMSISReading | None` — 解析 MSIS JSON。支持 base64 / list / dict 格式的 `beam_data`。

---

### `ogm/transforms.py` — 坐标变换与波束足迹

提供体坐标→世界坐标、世界→栅格索引的转换，以及 SBES/MSIS 波束足迹采样。

**函数**

- `body_to_world(bx: float, by: float, pose_x: float, pose_y: float, heading: float) -> tuple[float, float]`
  将体坐标系点旋转平移到世界坐标系。

- `world_to_grid(wx: float, wy: float, cell_size: float) -> tuple[int, int]`
  世界坐标 → 栅格索引（floor 取整）。

- `grid_to_world(gx: int, gy: int, cell_size: float) -> tuple[float, float]`
  栅格索引 → 世界坐标（单元中心）。

- `sbes_footprint(data: SBESData, cell_size: float, aperture_deg: float = 0.0) -> tuple[list[tuple[int, int]], list[tuple[tuple[int, int], float]]]`
  沿 SBES 波束采样，返回 `(free_cells, hit_cells_with_weight)`。以 `cell_size` 为步长从原点到量程逐步采样空闲格；末端命中点按 `aperture_deg` 进行垂直于波束方向的高斯展宽，每个命中格携带权重。

- `msis_footprint(data: MSISData, cell_size: float, aperture_deg: float = 0.0, bin_threshold: int = 0) -> tuple[list[tuple[int, int]], list[tuple[tuple[int, int], float]]]`
  沿 MSIS 波束采样，返回 `(free_cells, hit_cells_with_weight)`。强度 ≥ `bin_threshold` 的首个 bin 之前的 bin 标记为空闲；该 bin 及之后高于阈值的 bin 各自进行高斯展宽，携带权重。

---

### `ogm/quadtree.py` — 动态四叉树

稀疏存储对数几率值的线程安全动态四叉树。使用整数栅格索引，叶节点大小 = 1 格。根节点按需扩展（2 的幂对齐）。所有公共方法由 `RLock` 保护。

**类**

#### `QuadTree`

```python
QuadTree(cell_size: float)
```

| 参数 | 说明 |
|------|------|
| `cell_size` | 栅格单元边长（米），用于世界坐标 ↔ 索引转换 |

公共方法：

- `set_cell(gx: int, gy: int, value: float) -> None`
  按栅格索引设置对数几率值。

- `get_cell(gx: int, gy: int) -> float | None`
  按栅格索引获取对数几率值，未设置返回 `None`。

- `update_cell(gx: int, gy: int, delta: float, clamp: float = 10.0) -> None`
  对单元的对数几率累加 `delta`，结果钳位到 `[-clamp, clamp]`。

- `set(x: float, y: float, value: float) -> None`
  按世界坐标设置值（内部转换为栅格索引）。

- `get(x: float, y: float) -> float | None`
  按世界坐标获取值。

- `query_rect(xmin: float, ymin: float, xmax: float, ymax: float) -> list[tuple[int, int, float]]`
  矩形范围查询，返回 `(gx, gy, log_odds)` 列表。

- `query_circle(cx: float, cy: float, r: float) -> list[tuple[int, int, float]]`
  圆形范围查询，返回 `(gx, gy, log_odds)` 列表。AABB 剪枝后对叶节点做精确距离检测。

---

### `ogm/grid.py` — 占据栅格

贝叶斯对数几率更新的核心模块，封装 `QuadTree`，将传感器观测转化为栅格概率。

**模块级函数**（内部）

- `_log_odds(p: float) -> float` — 概率 → 对数几率。
- `_probability(lo: float) -> float` — 对数几率 → 概率。

**类**

#### `OccupancyGrid`

```python
OccupancyGrid(cell_size: float)
```

| 参数 | 说明 |
|------|------|
| `cell_size` | 栅格分辨率（米） |

公共方法：

- `update_pose(odom: OdometryData) -> None` — 更新最新位姿（线程安全）。
- `get_pose() -> OdometryData | None` — 获取最新位姿。
- `update_sbes(data: SBESData, confidence: float, aperture_deg: float = 0.0) -> None` — 处理一次 SBES 测量，按给定置信度和波束展宽角计算对数几率增量，更新空闲/命中格。
- `update_msis(data: MSISData, confidence: float = 0.9, aperture_deg: float = 0.0, bin_threshold: int = 0) -> None` — 处理一条 MSIS 波束，按置信度、展宽角和 bin 阈值更新空闲/命中格。
- `get_probability(gx: int, gy: int) -> float` — 查询单格占据概率，未知返回 `0.5`。
- `query_rect(xmin: float, ymin: float, xmax: float, ymax: float) -> list[tuple[int, int, float]]` — 矩形区域查询，返回 `(gx, gy, probability)`。
- `query_circle(cx: float, cy: float, r: float) -> list[tuple[int, int, float]]` — 圆形区域查询，返回 `(gx, gy, probability)`。

---

### `ogm/contour.py` — 轮廓提取

从栅格中提取占据边界点。

**函数**

- `extract_contour(grid: OccupancyGrid, cx: float, cy: float, r: float, threshold: float = 0.65) -> list[tuple[float, float]]`

  在圆形区域 `(cx, cy, r)` 内提取轮廓点（世界坐标）。判定规则：概率 ≥ `threshold` 且至少一个 4-邻域概率 < `threshold` 或未知。

---

### `ogm/colormap.py` — Jet 色表

将占据概率映射为 Jet 彩虹色，用于可视化。

**常量**

- `JET_LUT: list[tuple[int, int, int]]` — 预计算的 256 条 RGB 查找表。

**函数**

- `probability_to_color(prob: float) -> tuple[int, int, int]` — 概率 `[0, 1]` → Jet RGB。

---

### `ogm/comm.py` — pynng 通信

管理 4 个 Sub0 订阅线程和 1 个 Rep0 轮廓服务线程。

**类**

#### `SensorSubscriber(threading.Thread)`

订阅单个传感器地址，解析后调用处理函数。每个传感器使用独立地址区分，无话题前缀。

```python
SensorSubscriber(
    name: str,
    sensor_cfg: SensorConfig,
    parse_fn: Callable[[bytes], object | None],
    handle_fn: Callable[[object], None],
    stop_event: threading.Event,
)
```

- `run() -> None` — 线程主循环：`dial` → `subscribe(b"")` → `recv` → `parse_fn` → `handle_fn`。

#### `ContourServer(threading.Thread)`

Rep0 服务，接收 JSON 圆形查询，返回轮廓点。

```python
ContourServer(
    address: str,
    grid: OccupancyGrid,
    threshold: float,
    stop_event: threading.Event,
)
```

请求格式：`{"x": float, "y": float, "r": float}`
响应格式：`{"points": [[x1, y1], [x2, y2], ...]}`

公共方法：
- `get_latest_contour() -> list[tuple[float, float]]` — 获取最近一次计算的轮廓点（可视化用，线程安全）。
- `run() -> None` — 线程主循环。

**工厂函数**

- `_make_sbes_handler(confidence: float, beam_angle_deg: float, aperture_deg: float) -> Callable` — 创建 SBES handler 闭包，接收 `RawSBESReading`，通过 `grid.get_pose()` 补齐位置和世界航向后调用 `grid.update_sbes`。
- `start_subscribers(config: AppConfig, grid: OccupancyGrid, stop_event: threading.Event) -> list[SensorSubscriber]`
  创建并启动 4 个传感器订阅线程（odometry + msis + sbes_left + sbes_right）。SBES 和 MSIS handler 在收到原始读数后，从 `grid.get_pose()` 获取当前位姿组装完整数据。

- `start_contour_server(config: AppConfig, grid: OccupancyGrid, stop_event: threading.Event) -> ContourServer`
  创建并启动轮廓服务线程。

---

### `ogm/visualizer.py` — PyGame 可视化

实时渲染占据栅格、AUV 位姿和轮廓点，支持鼠标拖拽平移和滚轮缩放。

**常量**

| 名称 | 值 | 说明 |
|------|----|------|
| `BG_COLOR` | `(30, 30, 30)` | 背景色 |
| `AUV_COLOR` | `(0, 220, 0)` | AUV 三角形颜色 |
| `CONTOUR_COLOR` | `(255, 0, 255)` | 轮廓点颜色 |
| `GRID_LINE_COLOR` | `(60, 60, 60)` | 栅格线颜色 |

**类**

#### `Visualizer`

```python
Visualizer(
    width: int,
    height: int,
    fps: int,
    grid: OccupancyGrid,
    contour_server: ContourServer | None = None,
    camera_x: float = 120.0,
    camera_y: float = 0.0,
)
```

公共方法：
- `world_to_screen(wx: float, wy: float) -> tuple[int, int]` — 世界坐标 → 屏幕像素（Y 翻转）。
- `screen_to_world(sx: int, sy: int) -> tuple[float, float]` — 屏幕像素 → 世界坐标。
- `run(stop_event: threading.Event) -> None` — 主渲染循环（需在主线程运行）。支持拖拽平移、滚轮缩放、ESC/关闭退出。渲染内容包括栅格单元、AUV 位姿、轮廓点、原点十字线和 XY 刻度尺。

私有方法：
- `_viewport_world_bounds() -> tuple[float, float, float, float]` — 返回当前视口可见世界范围 `(xmin, ymin, xmax, ymax)`。
- `_draw_rulers(screen, font) -> None` — 绘制顶部 X 轴和左侧 Y 轴刻度尺，刻度间距根据缩放级别自动调整。
- `_draw_auv(screen, wx, wy, heading) -> None` — 在指定世界坐标绘制 AUV 方向三角形。

---

### `ogm/__init__.py` — 包声明

模块文档字符串：`"2D Occupancy Grid Map with quadtree storage."`

---

## 核心算法概要

### 对数几率贝叶斯更新（Log-Odds Bayes）

每个栅格单元存储对数几率 `l`，初始为 `0`（对应概率 `0.5`，即"未知"）。

```
l(p)   = log(p / (1 - p))
p(l)   = 1 / (1 + exp(-l))
l_new  = l_old + l(p_obs)
```

- SBES：空闲格 `delta = l(1 - confidence)`；命中格按高斯权重调制 `delta = l(0.5 + (confidence - 0.5) * weight)`，其中 `weight ∈ [0.01, 1.0]` 来自波束展宽的高斯分布。`confidence` 由各 SBES 传感器配置独立指定。
- MSIS：空闲格 `delta = l(0.3)`；命中格同样按权重调制 `delta = l(0.5 + (confidence - 0.5) * weight)`。仅强度 ≥ `bin_threshold` 的 bin 被视为命中。
- 累加结果钳位到 `[-10, +10]` 防止数值饱和。

### 四叉树（QuadTree）

- 叶节点大小 = 1 格，每个叶存储一个 `float`（对数几率）。
- 根节点按需动态扩展（2 的幂对齐），覆盖所有已插入单元。
- 矩形查询：递归裁剪节点 AABB 与查询框的交集。
- 圆形查询：先 AABB-圆相交裁剪，叶节点再做精确中心距离检测。

### 波束足迹采样（Beam Footprint）

**SBES**：沿波束方向以 `cell_size` 为步长从发射点采样到量程端，采样路径上的格标记为空闲。末端命中点按 `aperture_deg` 进行垂直于波束方向的高斯展宽（`sigma = range * tan(aperture/2)`），展宽范围内各格按高斯权重标记为命中。

**MSIS**：将波束按 bin 等分。强度 ≥ `bin_threshold`（绝对阈值，uint8）的首个 bin 之前均为空闲格；该 bin 及之后高于阈值的 bin 各自按 `aperture_deg` 进行高斯展宽，携带权重标记为命中格。

### 轮廓提取（Contour）

在圆形查询区域内，筛选概率 ≥ 阈值的"已占据"格。若其 4-邻域中存在概率 < 阈值或未知的格，则认定为边界点，将其单元中心坐标输出。

---

## TODO

（暂无）
