#!/usr/bin/env python3
"""
Generate a complex terrain heightmap for contour tracking testing.

Features:
- Concave regions (内凹区域)
- Convex regions (外凸区域)
- Curved edges (曲线边缘)
- Sharp corners (尖角)

The terrain is designed for AUV contour tracking (NED coordinate system):
- White (255) = Low height value = Obstacle/Island (AUV needs to navigate around)
- Black (0) = High height value = Deep water (navigable area)

In Stonefish: heightValue = (1.0 - pixel/255) * maxHeight
- pixel=0 (black) -> height=maxHeight (deep, navigable)
- pixel=255 (white) -> height=0 (shallow, obstacle)

Usage:
    python generate_complex_terrain.py

Output:
    complex_terrain.png - 512x512 grayscale heightmap
"""

import numpy as np
from PIL import Image
import math


def create_complex_terrain(size=512):
    """
    Create a complex terrain heightmap.

    The design creates a single connected island with complex coastline:
    - Main body is a deformed ellipse
    - Added protrusions and indentations (all connected to main body)
    - Curved sections and sharp corners

    Args:
        size: Image size in pixels (square)

    Returns:
        numpy array of shape (size, size) with values 0-255
    """
    # Create coordinate grid centered at (0, 0)
    # Range: [-1, 1] for both x and y
    x = np.linspace(-1, 1, size)
    y = np.linspace(-1, 1, size)
    X, Y = np.meshgrid(x, y)

    # Convert to polar coordinates
    R = np.sqrt(X**2 + Y**2)
    Theta = np.arctan2(Y, X)

    # Base shape: deformed circle with varying radius
    # r(theta) = base_radius + sum of sinusoidal deformations
    base_radius = 0.45

    # Add multiple frequency components for complex shape
    r_boundary = (
        base_radius
        # Low frequency: overall elongation
        + 0.08 * np.cos(2 * Theta)
        # Medium frequency: major protrusions/indentations
        + 0.06 * np.cos(3 * Theta - 0.5)
        + 0.05 * np.sin(4 * Theta + 1.0)
        # Higher frequency: smaller features
        + 0.04 * np.cos(5 * Theta)
        + 0.03 * np.sin(7 * Theta - 0.3)
        # Very high frequency: fine detail
        + 0.02 * np.cos(11 * Theta + 0.7)
    )

    # Create base terrain mask
    terrain = np.zeros((size, size), dtype=np.float32)
    inside = R < r_boundary
    terrain[inside] = 1.0  # Inside = high terrain (obstacle)

    # Add specific features (all connected to main body)

    # Feature 1: Sharp pointed protrusion (North) - 尖锐外凸
    # Connected to main body via a neck
    for i in range(size):
        for j in range(size):
            dx = X[i, j]
            dy = Y[i, j] - 0.45  # Start from main body edge
            # Narrow triangle shape tapering to a point
            if dy > 0 and abs(dx) < 0.12 * (1 - dy/0.35) and dy < 0.35:
                terrain[i, j] = 1.0

    # Feature 2: Deep concave bay (East) - 深凹海湾
    cx, cy = 0.35, 0.0  # Center of bay
    bay_radius = 0.15
    for i in range(size):
        for j in range(size):
            dx = X[i, j] - cx
            dy = Y[i, j] - cy
            d = np.sqrt(dx**2 + dy**2)
            # Carve out a circular bay
            if d < bay_radius and X[i, j] > 0.25:
                terrain[i, j] = 0.0

    # Feature 3: Narrow channel/fjord (South-West) - 狭窄峡湾
    for i in range(size):
        for j in range(size):
            x_val = X[i, j]
            y_val = Y[i, j]
            # Narrow channel cutting into the terrain
            if -0.5 < x_val < -0.2 and -0.08 < y_val + 0.3 + 0.1*x_val < 0.08:
                if x_val > -0.45:
                    terrain[i, j] = 0.0

    # Feature 4: Curved peninsula (South-East) - 弯曲半岛
    # Make sure it connects to main body
    for i in range(size):
        for j in range(size):
            x_val = X[i, j]
            y_val = Y[i, j]
            # Arc-shaped protrusion - connected version
            arc_cx, arc_cy = 0.15, -0.45
            arc_r = np.sqrt((x_val - arc_cx)**2 + (y_val - arc_cy)**2)
            arc_theta = np.arctan2(y_val - arc_cy, x_val - arc_cx)
            if 0.08 < arc_r < 0.22 and -2.3 < arc_theta < -0.7:
                terrain[i, j] = 1.0

    # Feature 5: Stepped coastline (North-West) - 阶梯海岸
    # Connect steps to main body
    for i in range(size):
        for j in range(size):
            x_val = X[i, j]
            y_val = Y[i, j]
            # Create connected steps
            if x_val < -0.25 and y_val > 0.15:
                # Step 1 (connected to main)
                if -0.45 < x_val < -0.30 and 0.15 < y_val < 0.38:
                    terrain[i, j] = 1.0
                # Step 2
                if -0.55 < x_val < -0.40 and 0.22 < y_val < 0.42:
                    terrain[i, j] = 1.0
                # Step 3
                if -0.62 < x_val < -0.50 and 0.28 < y_val < 0.45:
                    terrain[i, j] = 1.0

    # Feature 6: 90-degree corner (L-shape protrusion) - 直角凸起
    # Connected to main body
    for i in range(size):
        for j in range(size):
            x_val = X[i, j]
            y_val = Y[i, j]
            # L-shaped protrusion in NE - ensure connection
            if (0.35 < x_val < 0.55 and 0.15 < y_val < 0.32) or \
               (0.35 < x_val < 0.48 and 0.32 < y_val < 0.48):
                terrain[i, j] = 1.0

    # Smooth the terrain slightly to avoid jagged edges
    from scipy import ndimage
    terrain = ndimage.gaussian_filter(terrain, sigma=1.5)

    # Threshold to create sharp boundary
    terrain = (terrain > 0.5).astype(np.float32)

    # Keep only the largest connected component (remove small isolated regions)
    terrain_binary = terrain.astype(np.uint8)
    labeled, num_features = ndimage.label(terrain_binary)
    if num_features > 1:
        # Find the largest component
        component_sizes = ndimage.sum(terrain_binary, labeled, range(1, num_features + 1))
        largest_component = np.argmax(component_sizes) + 1
        terrain = (labeled == largest_component).astype(np.float32)
        print(f"  Removed {num_features - 1} small isolated region(s)")

    # Apply slight smoothing for natural look
    terrain = ndimage.gaussian_filter(terrain, sigma=0.8)

    # Convert to 8-bit grayscale
    # In Stonefish (NED coords): heightValue = (1.0 - pixel/255) * maxHeight
    #   pixel=0 (black) -> height=maxHeight -> deep water (navigable)
    #   pixel=255 (white) -> height=0 -> shallow/obstacle (need to avoid)
    # So: terrain=1 (obstacle) -> white (255), terrain=0 (water) -> black (0)
    heightmap = (terrain * 255).astype(np.uint8)

    return heightmap


def main():
    print("Generating complex terrain heightmap...")

    # Generate terrain
    heightmap = create_complex_terrain(size=512)

    # Save as PNG
    img = Image.fromarray(heightmap, mode='L')
    output_path = "complex_terrain.png"
    img.save(output_path)

    print(f"Saved: {output_path}")
    print(f"Size: {heightmap.shape[0]}x{heightmap.shape[1]} pixels")
    print(f"Value range: {heightmap.min()} - {heightmap.max()}")
    print()
    print("Terrain features (single connected region):")
    print("  - Central island with complex coastline")
    print("  - Sharp pointed protrusion (North) - 尖锐外凸")
    print("  - Deep concave bay (East) - 深凹海湾")
    print("  - Narrow channel/fjord (South-West) - 狭窄峡湾")
    print("  - Curved peninsula (South-East) - 弯曲半岛")
    print("  - Stepped coastline (North-West) - 阶梯海岸")
    print("  - L-shaped 90° corner (North-East) - 直角凸起")
    print()
    print("Usage in SCN file:")
    print('  <static name="Terrain" type="terrain">')
    print('    <material name="Rock"/>')
    print('    <look name="seabed"/>')
    print('    <world_transform rpy="0.0 0.0 0.0" xyz="0.0 0.0 10.0"/>')
    print('    <height_map filename="environment/complex_terrain.png"/>')
    print('    <dimensions scalex="1.0" scaley="1.0" height="20.0"/>')
    print('  </static>')


if __name__ == "__main__":
    main()
