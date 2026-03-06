"""Dynamic quadtree for sparse log-odds storage.

Uses integer grid indices internally to avoid floating-point drift.
Leaf size = 1 grid cell. Root expands dynamically (power-of-2 aligned).
All public methods are protected by an RLock for thread safety.
"""

from __future__ import annotations

import threading


class _Node:
    """Internal quadtree node.

    Covers grid cells [ox, ox+size) x [oy, oy+size).
    A leaf has size=1 and stores a log-odds value.
    """

    __slots__ = ("ox", "oy", "size", "value", "children")

    def __init__(self, ox: int, oy: int, size: int) -> None:
        self.ox = ox
        self.oy = oy
        self.size = size
        self.value: float | None = None
        self.children: list[_Node | None] = [None, None, None, None]


class QuadTree:
    """Thread-safe dynamic quadtree storing log-odds values at cell resolution.

    Grid cell (gx, gy) maps to world center ((gx+0.5)*cell_size, (gy+0.5)*cell_size).
    """

    def __init__(self, cell_size: float) -> None:
        self.cell_size = cell_size
        self._lock = threading.RLock()
        # Initial root covers [0, 1) x [0, 1) — will expand as needed
        self._root = _Node(0, 0, 1)

    @staticmethod
    def _contains(node: _Node, gx: int, gy: int) -> bool:
        return (node.ox <= gx < node.ox + node.size and
                node.oy <= gy < node.oy + node.size)

    def _expand_root(self, gx: int, gy: int) -> None:
        """Expand root until it contains (gx, gy)."""
        while not self._contains(self._root, gx, gy):
            old = self._root
            new_size = old.size * 2
            # Choose new origin so the old root is one of the 4 children
            new_ox = old.ox - old.size if gx < old.ox else old.ox
            new_oy = old.oy - old.size if gy < old.oy else old.oy
            new_root = _Node(new_ox, new_oy, new_size)
            # Place old root as appropriate child
            q = QuadTree._quadrant_of(new_root, old.ox, old.oy)
            new_root.children[q] = old
            self._root = new_root

    @staticmethod
    def _quadrant_of(parent: _Node, gx: int, gy: int) -> int:
        """Return child quadrant index (0-3) for point (gx, gy) within parent."""
        half = parent.size >> 1
        ix = 0 if gx < parent.ox + half else 1
        iy = 0 if gy < parent.oy + half else 1
        return iy * 2 + ix

    @staticmethod
    def _child_origin(parent: _Node, q: int) -> tuple[int, int]:
        half = parent.size >> 1
        ox = parent.ox if (q & 1) == 0 else parent.ox + half
        oy = parent.oy if (q & 2) == 0 else parent.oy + half
        return ox, oy

    def _set(self, node: _Node, gx: int, gy: int, value: float) -> None:
        if node.size == 1:
            node.value = value
            return
        q = self._quadrant_of(node, gx, gy)
        child = node.children[q]
        if child is None:
            ox, oy = self._child_origin(node, q)
            child = _Node(ox, oy, node.size >> 1)
            node.children[q] = child
        self._set(child, gx, gy, value)

    def _get_node(self, node: _Node, gx: int, gy: int) -> _Node | None:
        if node.size == 1:
            return node
        q = self._quadrant_of(node, gx, gy)
        child = node.children[q]
        if child is None:
            return None
        return self._get_node(child, gx, gy)

    # ---- Public cell-level API ----

    def set_cell(self, gx: int, gy: int, value: float) -> None:
        """Set log-odds by grid index."""
        with self._lock:
            if not self._contains(self._root, gx, gy):
                self._expand_root(gx, gy)
            self._set(self._root, gx, gy, value)

    def get_cell(self, gx: int, gy: int) -> float | None:
        """Get log-odds by grid index, or None if unset."""
        with self._lock:
            if not self._contains(self._root, gx, gy):
                return None
            node = self._get_node(self._root, gx, gy)
            if node is None:
                return None
            return node.value

    def update_cell(self, gx: int, gy: int, delta: float, clamp: float = 10.0) -> None:
        """Add delta to the log-odds of cell (gx, gy), clamping to [-clamp, clamp]."""
        with self._lock:
            if not self._contains(self._root, gx, gy):
                self._expand_root(gx, gy)
            node = self._get_node(self._root, gx, gy)
            if node is not None and node.value is not None:
                new_val = max(-clamp, min(clamp, node.value + delta))
            else:
                new_val = max(-clamp, min(clamp, delta))
            self._set(self._root, gx, gy, new_val)

    def update_cells_batch(self, updates: list[tuple[int, int, float]], clamp: float = 10.0) -> None:
        """Batch update: single lock acquisition for all cells."""
        with self._lock:
            for gx, gy, delta in updates:
                if not self._contains(self._root, gx, gy):
                    self._expand_root(gx, gy)
                node = self._get_node(self._root, gx, gy)
                if node is not None and node.value is not None:
                    new_val = max(-clamp, min(clamp, node.value + delta))
                else:
                    new_val = max(-clamp, min(clamp, delta))
                self._set(self._root, gx, gy, new_val)

    # ---- World-coordinate API (for convenience) ----

    def set(self, x: float, y: float, value: float) -> None:
        """Set log-odds at world coordinate (x, y)."""
        import math
        gx = int(math.floor(x / self.cell_size))
        gy = int(math.floor(y / self.cell_size))
        self.set_cell(gx, gy, value)

    def get(self, x: float, y: float) -> float | None:
        """Get log-odds at world coordinate (x, y)."""
        import math
        gx = int(math.floor(x / self.cell_size))
        gy = int(math.floor(y / self.cell_size))
        return self.get_cell(gx, gy)

    # ---- Spatial queries ----

    def _collect_rect(
        self,
        node: _Node,
        gx_min: int,
        gy_min: int,
        gx_max: int,
        gy_max: int,
        results: list[tuple[int, int, float]],
    ) -> None:
        # Node covers [node.ox, node.ox+node.size) x [node.oy, node.oy+node.size)
        # Query covers [gx_min, gx_max] x [gy_min, gy_max] (inclusive)
        if (node.ox + node.size <= gx_min or node.ox > gx_max or
                node.oy + node.size <= gy_min or node.oy > gy_max):
            return

        if node.size == 1:
            if node.value is not None:
                results.append((node.ox, node.oy, node.value))
            return

        for child in node.children:
            if child is not None:
                self._collect_rect(child, gx_min, gy_min, gx_max, gy_max, results)

    def query_rect(self, xmin: float, ymin: float, xmax: float, ymax: float) -> list[tuple[int, int, float]]:
        """Return all cells (gx, gy, log_odds) within the world-coord rectangle."""
        import math
        gx_min = int(math.floor(xmin / self.cell_size))
        gy_min = int(math.floor(ymin / self.cell_size))
        gx_max = int(math.floor(xmax / self.cell_size))
        gy_max = int(math.floor(ymax / self.cell_size))
        with self._lock:
            results: list[tuple[int, int, float]] = []
            self._collect_rect(self._root, gx_min, gy_min, gx_max, gy_max, results)
            return results

    def _collect_circle(
        self,
        node: _Node,
        cx: float,
        cy: float,
        r: float,
        r2: float,
        cell_size: float,
        results: list[tuple[int, int, float]],
    ) -> None:
        # AABB of this node in world coordinates
        wx_min = node.ox * cell_size
        wy_min = node.oy * cell_size
        wx_max = (node.ox + node.size) * cell_size
        wy_max = (node.oy + node.size) * cell_size

        # Closest point on AABB to circle center
        closest_x = max(wx_min, min(cx, wx_max))
        closest_y = max(wy_min, min(cy, wy_max))
        dx = closest_x - cx
        dy = closest_y - cy
        if dx * dx + dy * dy > r2:
            return

        if node.size == 1:
            if node.value is not None:
                # Check cell center within circle
                cell_cx = (node.ox + 0.5) * cell_size
                cell_cy = (node.oy + 0.5) * cell_size
                ddx = cell_cx - cx
                ddy = cell_cy - cy
                if ddx * ddx + ddy * ddy <= r2:
                    results.append((node.ox, node.oy, node.value))
            return

        for child in node.children:
            if child is not None:
                self._collect_circle(child, cx, cy, r, r2, cell_size, results)

    def query_circle(self, cx: float, cy: float, r: float) -> list[tuple[int, int, float]]:
        """Return all cells (gx, gy, log_odds) within a circle (world coords)."""
        with self._lock:
            results: list[tuple[int, int, float]] = []
            r2 = r * r
            self._collect_circle(self._root, cx, cy, r, r2, self.cell_size, results)
            return results

    # ---- Pruning ----

    @staticmethod
    def _count_leaves(node: _Node) -> int:
        """Recursively count leaf nodes (cells with values) in the subtree."""
        if node.size == 1:
            return 1 if node.value is not None else 0
        total = 0
        for child in node.children:
            if child is not None:
                total += QuadTree._count_leaves(child)
        return total

    def count_cells(self) -> int:
        """Count total grid cells stored in the tree."""
        with self._lock:
            return self._count_leaves(self._root)

    @staticmethod
    def _prune_node(
        node: _Node,
        gx_min: int,
        gy_min: int,
        gx_max: int,
        gy_max: int,
    ) -> tuple[int, bool]:
        """Recursively prune nodes outside the keep rectangle.

        The keep rectangle is [gx_min, gx_max] x [gy_min, gy_max] (inclusive).
        Returns (removed_count, is_empty) where is_empty means this node
        can be discarded by its parent.
        """
        n_ox = node.ox
        n_oy = node.oy
        n_end_x = n_ox + node.size   # exclusive upper bound
        n_end_y = n_oy + node.size

        # Fully inside the keep rect → skip (nothing to prune)
        if n_ox >= gx_min and n_end_x - 1 <= gx_max and n_oy >= gy_min and n_end_y - 1 <= gy_max:
            return 0, False

        # Fully outside the keep rect → remove entire subtree
        if n_end_x <= gx_min or n_ox > gx_max or n_end_y <= gy_min or n_oy > gy_max:
            removed = QuadTree._count_leaves(node)
            return removed, True

        # Leaf node that is outside (partial overlap at leaf level)
        if node.size == 1:
            if node.value is not None:
                return 1, True
            return 0, True

        # Partial overlap → recurse into children
        removed = 0
        for q in range(4):
            child = node.children[q]
            if child is None:
                continue
            child_removed, child_empty = QuadTree._prune_node(
                child, gx_min, gy_min, gx_max, gy_max,
            )
            removed += child_removed
            if child_empty:
                node.children[q] = None

        # If all children are now None, this node is empty
        is_empty = all(c is None for c in node.children)
        return removed, is_empty

    def prune_outside_rect(
        self,
        gx_min: int,
        gy_min: int,
        gx_max: int,
        gy_max: int,
    ) -> int:
        """Remove all cells outside the keep rectangle [gx_min..gx_max] x [gy_min..gy_max].

        Returns the number of removed cells.
        """
        with self._lock:
            removed, _ = self._prune_node(
                self._root, gx_min, gy_min, gx_max, gy_max,
            )
            return removed
