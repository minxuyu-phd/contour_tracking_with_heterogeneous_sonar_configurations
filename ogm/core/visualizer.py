"""PyGame visualizer: drag/zoom, grid Jet coloring, AUV triangle, contour highlights."""

from __future__ import annotations

import math
from typing import TYPE_CHECKING

import pygame

from .colormap import probability_to_color

if TYPE_CHECKING:
    from .comm import ContourServer
    from .grid import OccupancyGrid


BG_COLOR = (30, 30, 30)
AUV_COLOR = (0, 220, 0)
CONTOUR_COLOR = (255, 0, 255)
GRID_LINE_COLOR = (60, 60, 60)


class Visualizer:
    """PyGame-based 2D occupancy grid visualizer."""

    def __init__(
        self,
        width: int,
        height: int,
        fps: int,
        grid: OccupancyGrid,
        contour_server: ContourServer | None = None,
        camera_x: float = 120.0,
        camera_y: float = 0.0,
        render_threshold: float = 0.0,
        grid_opacity: float = 0.7,
        follow_auv: bool = False,
    ) -> None:
        self.width = width
        self.height = height
        self.fps = fps
        self.grid = grid
        self.contour_server = contour_server
        self.render_threshold = render_threshold
        self.grid_opacity = grid_opacity
        self.follow_auv = follow_auv

        # Camera state (world coordinates of screen center)
        self.cam_x: float = camera_x
        self.cam_y: float = camera_y
        self.zoom: float = 50.0  # pixels per meter
        self.min_zoom: float = 5.0
        self.max_zoom: float = 500.0

        # Drag state
        self._dragging = False
        self._drag_start: tuple[int, int] = (0, 0)
        self._cam_start: tuple[float, float] = (0.0, 0.0)

    def world_to_screen(self, wx: float, wy: float) -> tuple[int, int]:
        """Convert world coordinates to screen pixel coordinates (Y-down positive)."""
        sx = int((wx - self.cam_x) * self.zoom + self.width / 2)
        sy = int((wy - self.cam_y) * self.zoom + self.height / 2)
        return sx, sy

    def screen_to_world(self, sx: int, sy: int) -> tuple[float, float]:
        """Convert screen pixel coordinates to world coordinates."""
        wx = (sx - self.width / 2) / self.zoom + self.cam_x
        wy = (sy - self.height / 2) / self.zoom + self.cam_y
        return wx, wy

    def _viewport_world_bounds(self) -> tuple[float, float, float, float]:
        """Return (xmin, ymin, xmax, ymax) of the visible world area."""
        x0, y0 = self.screen_to_world(0, self.height)
        x1, y1 = self.screen_to_world(self.width, 0)
        return min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)

    def run(self, stop_event) -> None:
        """Main visualization loop. Call from the main thread."""
        pygame.init()
        screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("OGM2D Visualizer")
        clock = pygame.time.Clock()

        grid_surface = pygame.Surface((self.width, self.height))

        ruler_font = pygame.font.SysFont("monospace", 12)

        running = True
        while running and not stop_event.is_set():
            # --- Events ---
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    stop_event.set()
                    running = False
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    if event.button == 1 and not self.follow_auv:
                        self._dragging = True
                        self._drag_start = event.pos
                        self._cam_start = (self.cam_x, self.cam_y)
                    elif event.button == 4:  # scroll up
                        self.zoom = min(self.max_zoom, self.zoom * 1.15)
                    elif event.button == 5:  # scroll down
                        self.zoom = max(self.min_zoom, self.zoom / 1.15)
                elif event.type == pygame.MOUSEBUTTONUP:
                    if event.button == 1:
                        self._dragging = False
                elif event.type == pygame.MOUSEMOTION:
                    if self._dragging and not self.follow_auv:
                        dx = event.pos[0] - self._drag_start[0]
                        dy = event.pos[1] - self._drag_start[1]
                        self.cam_x = self._cam_start[0] - dx / self.zoom
                        self.cam_y = self._cam_start[1] - dy / self.zoom
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        stop_event.set()
                        running = False

            if not running:
                break

            # Follow AUV: lock camera to AUV position
            if self.follow_auv:
                pose = self.grid.get_pose()
                if pose is not None:
                    self.cam_x = pose.x
                    self.cam_y = pose.y

            # --- Render ---
            screen.fill(BG_COLOR)

            # Draw grid cells (semi-transparent via overlay surface)
            xmin, ymin, xmax, ymax = self._viewport_world_bounds()
            # Add margin
            margin = self.grid.cell_size * 2
            cells = self.grid.query_rect(xmin - margin, ymin - margin, xmax + margin, ymax + margin)

            cs = self.grid.cell_size
            pixel_size = max(1, int(cs * self.zoom))

            grid_surface.fill(BG_COLOR)
            grid_surface.set_colorkey(BG_COLOR)

            threshold = self.render_threshold
            for gx, gy, prob in cells:
                if prob < threshold:
                    continue
                wx = gx * cs
                wy = gy * cs
                sx, sy = self.world_to_screen(wx, wy)
                color = probability_to_color(prob)
                pygame.draw.rect(grid_surface, color, (sx, sy, pixel_size, pixel_size))

            grid_surface.set_alpha(int(self.grid_opacity * 255))
            screen.blit(grid_surface, (0, 0))

            # Draw AUV
            pose = self.grid.get_pose()
            if pose is not None:
                self._draw_auv(screen, pose.x, pose.y, pose.heading)

            # Draw contour points
            if self.contour_server is not None:
                points = self.contour_server.get_latest_contour()
                for wx, wy in points:
                    sx, sy = self.world_to_screen(wx, wy)
                    pygame.draw.circle(screen, CONTOUR_COLOR, (sx, sy), 3)

            # Origin crosshair
            ox, oy = self.world_to_screen(0.0, 0.0)
            pygame.draw.line(screen, (100, 100, 100), (ox - 10, oy), (ox + 10, oy), 1)
            pygame.draw.line(screen, (100, 100, 100), (ox, oy - 10), (ox, oy + 10), 1)

            # Rulers
            self._draw_rulers(screen, ruler_font)

            pygame.display.flip()
            clock.tick(self.fps)

        pygame.quit()

    def _draw_rulers(self, screen: pygame.Surface, font: pygame.font.Font) -> None:
        """Draw X (top) and Y (left) rulers with world-coordinate tick marks."""
        ruler_color = (150, 150, 150)
        bg_color = (30, 30, 30, 180)
        top_h = 25
        left_w = 50

        # Pick tick interval: choose from candidates so screen spacing is 50-150 px
        candidates = [0.1, 0.2, 0.5, 1, 2, 5, 10, 20, 50, 100]
        interval = candidates[-1]
        for c in candidates:
            px_gap = c * self.zoom
            if 50 <= px_gap <= 150:
                interval = c
                break

        xmin, ymin, xmax, ymax = self._viewport_world_bounds()

        # --- Top ruler (X axis) ---
        top_bg = pygame.Surface((self.width, top_h), pygame.SRCALPHA)
        top_bg.fill(bg_color)
        screen.blit(top_bg, (0, 0))

        tick_start = math.floor(xmin / interval) * interval
        x = tick_start
        while x <= xmax:
            sx, _ = self.world_to_screen(x, 0)
            if left_w <= sx <= self.width:
                pygame.draw.line(screen, ruler_color, (sx, 0), (sx, top_h), 1)
                label = font.render(f"{x:.6g}", True, ruler_color)
                screen.blit(label, (sx + 2, 2))
            x += interval

        # --- Left ruler (Y axis) ---
        left_bg = pygame.Surface((left_w, self.height), pygame.SRCALPHA)
        left_bg.fill(bg_color)
        screen.blit(left_bg, (0, 0))

        tick_start = math.floor(ymin / interval) * interval
        y = tick_start
        while y <= ymax:
            _, sy = self.world_to_screen(0, y)
            if top_h <= sy <= self.height:
                pygame.draw.line(screen, ruler_color, (0, sy), (left_w, sy), 1)
                label = font.render(f"{y:.6g}", True, ruler_color)
                screen.blit(label, (2, sy + 2))
            y += interval

        # Corner block
        corner = pygame.Surface((left_w, top_h), pygame.SRCALPHA)
        corner.fill(bg_color)
        screen.blit(corner, (0, 0))

    def _draw_auv(self, screen: pygame.Surface, wx: float, wy: float, heading: float) -> None:
        """Draw AUV as a green triangle pointing in heading direction."""
        sx, sy = self.world_to_screen(wx, wy)
        size = 20

        # Triangle vertices: nose, left, right
        angles = [heading, heading + 2.5, heading - 2.5]
        lengths = [size, size * 0.6, size * 0.6]

        points = []
        for a, l in zip(angles, lengths):
            px = sx + l * math.cos(a)
            py = sy + l * math.sin(a)
            points.append((int(px), int(py)))

        pygame.draw.polygon(screen, (255, 255, 255), points)
        pygame.draw.polygon(screen, AUV_COLOR, points, 2)
