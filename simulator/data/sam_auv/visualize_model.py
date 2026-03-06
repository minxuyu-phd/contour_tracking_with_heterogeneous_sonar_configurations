#!/usr/bin/env python3
"""Visualize the SAM AUV model from sam_auv.xml using trimesh + pyglet (OpenGL)."""

# Fix for WSL2: conda's libstdc++ is too old for Mesa's LLVM, preload system version
import os
import sys

_SYSTEM_LIBSTDCXX = "/usr/lib/x86_64-linux-gnu/libstdc++.so.6"
if os.path.exists(_SYSTEM_LIBSTDCXX) and _SYSTEM_LIBSTDCXX not in os.environ.get("LD_PRELOAD", ""):
    os.environ["LD_PRELOAD"] = _SYSTEM_LIBSTDCXX
    os.execv(sys.executable, [sys.executable] + sys.argv)

import xml.etree.ElementTree as ET

import numpy as np
import pyglet
import trimesh
from scipy.spatial.transform import Rotation
from trimesh.viewer import SceneViewer

# Color definitions from scenario look names (RGB 0-1)
LOOK_COLORS = {
    "yellow": (1.0, 0.78, 0.0),
    "manipulator": (0.05, 0.05, 0.05),
    "black": (0.05, 0.05, 0.05),
    "gray": (0.3, 0.3, 0.3),
}
DEFAULT_COLOR = (0.7, 0.7, 0.7)

# Shorten hull for visualization: scale X axis of hull mesh, keep rear aligned
HULL_X_SCALE = 0.8

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MESH_DIR = os.path.join(SCRIPT_DIR, "mesh")


def rpy_xyz_to_matrix(rpy, xyz):
    """Convert RPY angles and XYZ translation to a 4x4 homogeneous transform matrix."""
    rot = Rotation.from_euler("xyz", rpy)
    mat = np.eye(4)
    mat[:3, :3] = rot.as_matrix()
    mat[:3, 3] = xyz
    return mat


def parse_transform(element):
    """Parse rpy and xyz attributes from an XML element into a 4x4 matrix."""
    rpy = [float(v) for v in element.get("rpy", "0 0 0").split()]
    xyz = [float(v) for v in element.get("xyz", "0 0 0").split()]
    return rpy_xyz_to_matrix(rpy, xyz)


def load_obj_mesh(filename, scale=1.0):
    """Load an OBJ mesh file."""
    filepath = os.path.join(MESH_DIR, os.path.basename(filename))
    mesh = trimesh.load(filepath, file_type="obj", process=True)
    if scale != 1.0:
        mesh.apply_scale(scale)
    return mesh


def apply_color(mesh, look_name):
    """Apply look color to a trimesh mesh."""
    color_rgb = LOOK_COLORS.get(look_name, DEFAULT_COLOR)
    rgba = [int(c * 255) for c in color_rgb] + [255]
    mesh.visual = trimesh.visual.ColorVisuals(mesh=mesh, face_colors=rgba)


def parse_compound_base_link(base_link_elem):
    """Parse a compound base_link element and return list of (mesh, name, look) tuples."""
    meshes = []

    for part in base_link_elem:
        if part.tag not in ("external_part", "internal_part"):
            continue

        name = part.get("name", "")
        part_type = part.get("type", "")
        look_elem = part.find("look")
        look_name = look_elem.get("name", "") if look_elem is not None else ""

        ct_elem = part.find("compound_transform")
        ct_mat = parse_transform(ct_elem) if ct_elem is not None else np.eye(4)

        mesh = None

        if part_type == "model":
            visual = part.find("visual")
            if visual is None:
                visual = part.find("physical")
            if visual is not None:
                mesh_elem = visual.find("mesh")
                origin_elem = visual.find("origin")
                filename = mesh_elem.get("filename")
                scale = float(mesh_elem.get("scale", "1.0"))
                origin_mat = parse_transform(origin_elem) if origin_elem is not None else np.eye(4)
                mesh = load_obj_mesh(filename, scale)

                # Shorten hull: compress X in local frame, adjust origin to keep rear aligned
                if name == "Hull" and HULL_X_SCALE != 1.0:
                    rear_x = mesh.vertices[:, 0].min()  # rear end in local frame
                    mesh.vertices[:, 0] *= HULL_X_SCALE
                    # Shift so rear stays at same local position
                    mesh.vertices[:, 0] += rear_x * (1.0 - HULL_X_SCALE)

                mesh.apply_transform(ct_mat @ origin_mat)

        elif part_type == "cylinder":
            dims = part.find("dimensions")
            radius = float(dims.get("radius"))
            height = float(dims.get("height"))
            origin_elem = part.find("origin")
            origin_mat = parse_transform(origin_elem) if origin_elem is not None else np.eye(4)
            mesh = trimesh.creation.cylinder(radius=radius, height=height, sections=32)
            mesh.apply_transform(ct_mat @ origin_mat)

        elif part_type == "box":
            dims = part.find("dimensions")
            xyz = [float(v) for v in dims.get("xyz").split()]
            origin_elem = part.find("origin")
            origin_mat = parse_transform(origin_elem) if origin_elem is not None else np.eye(4)
            mesh = trimesh.creation.box(extents=xyz)
            mesh.apply_transform(ct_mat @ origin_mat)

        elif part_type == "sphere":
            dims = part.find("dimensions")
            radius = float(dims.get("radius"))
            if radius < 0.005:
                continue
            origin_elem = part.find("origin")
            origin_mat = parse_transform(origin_elem) if origin_elem is not None else np.eye(4)
            mesh = trimesh.creation.uv_sphere(radius=radius, count=[16, 16])
            mesh.apply_transform(ct_mat @ origin_mat)

        if mesh is not None:
            meshes.append((mesh, name, look_name))

    return meshes


def parse_link(link_elem):
    """Parse a standalone link element and return (mesh, look_name). Mesh is in link-local frame."""
    link_type = link_elem.get("type", "")
    look_elem = link_elem.find("look")
    look_name = look_elem.get("name", "") if look_elem is not None else ""

    mesh = None

    if link_type == "model":
        visual = link_elem.find("visual")
        if visual is None:
            visual = link_elem.find("physical")
        if visual is not None:
            mesh_elem = visual.find("mesh")
            origin_elem = visual.find("origin")
            filename = mesh_elem.get("filename")
            scale = float(mesh_elem.get("scale", "1.0"))
            origin_mat = parse_transform(origin_elem) if origin_elem is not None else np.eye(4)
            mesh = load_obj_mesh(filename, scale)
            mesh.apply_transform(origin_mat)

    elif link_type == "box":
        dims = link_elem.find("dimensions")
        xyz = [float(v) for v in dims.get("xyz").split()]
        origin_elem = link_elem.find("origin")
        origin_mat = parse_transform(origin_elem) if origin_elem is not None else np.eye(4)
        mesh = trimesh.creation.box(extents=xyz)
        mesh.apply_transform(origin_mat)

    elif link_type == "cylinder":
        dims = link_elem.find("dimensions")
        radius = float(dims.get("radius"))
        height = float(dims.get("height"))
        origin_elem = link_elem.find("origin")
        origin_mat = parse_transform(origin_elem) if origin_elem is not None else np.eye(4)
        mesh = trimesh.creation.cylinder(radius=radius, height=height, sections=32)
        mesh.apply_transform(origin_mat)

    elif link_type == "sphere":
        dims = link_elem.find("dimensions")
        radius = float(dims.get("radius"))
        if radius < 0.005:
            return None, look_name
        origin_elem = link_elem.find("origin")
        origin_mat = parse_transform(origin_elem) if origin_elem is not None else np.eye(4)
        mesh = trimesh.creation.uv_sphere(radius=radius, count=[16, 16])
        mesh.apply_transform(origin_mat)

    return mesh, look_name


def main():
    xml_path = os.path.join(SCRIPT_DIR, "sam_auv.xml")
    tree = ET.parse(xml_path)
    root = tree.getroot()
    robot = root.find("robot")

    # Collect all (mesh, name, look_name) for rendering
    all_components = []

    # --- 1. Parse base_link (compound body at origin) ---
    base_link = robot.find("base_link")
    base_link_name = base_link.get("name")
    all_components.extend(parse_compound_base_link(base_link))

    # --- 2. Parse standalone links ---
    links = {}
    for link_elem in robot.findall("link"):
        link_name = link_elem.get("name")
        mesh, look_name = parse_link(link_elem)
        links[link_name] = (mesh, look_name)

    # --- 3. Build kinematic chain from joints ---
    link_world_transforms = {base_link_name: np.eye(4)}
    for joint_elem in robot.findall("joint"):
        parent_name = joint_elem.find("parent").get("name")
        child_name = joint_elem.find("child").get("name")
        joint_origin = parse_transform(joint_elem.find("origin"))
        parent_T = link_world_transforms.get(parent_name, np.eye(4))
        link_world_transforms[child_name] = parent_T @ joint_origin

    for link_name, (mesh, look_name) in links.items():
        if mesh is None:
            continue
        world_T = link_world_transforms.get(link_name, np.eye(4))
        mesh.apply_transform(world_T)
        all_components.append((mesh, link_name, look_name))

    # --- 4. Parse actuators with visual meshes ---
    for act_elem in robot.findall("actuator"):
        act_type = act_elem.get("type")

        if act_type == "thruster":
            link_name = act_elem.find("link").get("name")
            act_origin = parse_transform(act_elem.find("origin"))
            parent_T = link_world_transforms.get(link_name, np.eye(4))

            prop_elem = act_elem.find("propeller")
            if prop_elem is not None:
                mesh_elem = prop_elem.find("mesh")
                if mesh_elem is not None:
                    filename = mesh_elem.get("filename")
                    scale = float(mesh_elem.get("scale", "1.0"))
                    look_elem = prop_elem.find("look")
                    look_name = look_elem.get("name", "") if look_elem is not None else ""
                    prop_mesh = load_obj_mesh(filename, scale)
                    prop_mesh.apply_transform(parent_T @ act_origin)
                    all_components.append((prop_mesh, act_elem.get("name"), look_name))

        elif act_type == "vbs":
            link_name = act_elem.find("link").get("name")
            act_origin = parse_transform(act_elem.find("origin"))
            parent_T = link_world_transforms.get(link_name, np.eye(4))

            volume_elem = act_elem.find("volume")
            if volume_elem is not None:
                mesh_elem = volume_elem.find("mesh")
                if mesh_elem is not None:
                    filename = mesh_elem.get("filename")
                    vbs_mesh = load_obj_mesh(filename)
                    vbs_mesh.apply_transform(parent_T @ act_origin)
                    all_components.append((vbs_mesh, "VBS", ""))

    # --- 5. Build scene and render with OpenGL ---
    total_faces = sum(len(m.faces) for m, _, _ in all_components)
    print(f"Scene: {len(all_components)} components, {total_faces} total faces")

    scene = trimesh.Scene()
    for mesh, name, look_name in all_components:
        apply_color(mesh, look_name)
        scene.add_geometry(mesh, node_name=name)
        print(f"  - {name} (look={look_name}, faces={len(mesh.faces)})")

    save_path = os.path.join(SCRIPT_DIR, "sam_auv_view.png")

    class SAMViewer(SceneViewer):
        def on_key_press(self, symbol, modifiers):
            if symbol == pyglet.window.key.S:
                self._save_transparent_png()
            else:
                super().on_key_press(symbol, modifiers)

        def _save_transparent_png(self):
            import cv2

            # Capture GL color buffer
            manager = pyglet.image.get_buffer_manager()
            colorbuffer = manager.get_color_buffer()
            img_data = colorbuffer.get_image_data()
            raw = img_data.get_data("BGRA", img_data.width * 4)
            img = np.frombuffer(raw, dtype=np.uint8).reshape(img_data.height, img_data.width, 4)
            img = np.flipud(img)  # OpenGL is bottom-up

            # Crop non-white region
            bgr = img[:, :, :3]
            gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
            mask = gray < 255
            coords = np.argwhere(mask)
            if len(coords) == 0:
                print("No content to save")
                return
            y0, x0 = coords.min(axis=0)
            y1, x1 = coords.max(axis=0)
            pad = 10
            y0, x0 = max(0, y0 - pad), max(0, x0 - pad)
            y1, x1 = min(img.shape[0], y1 + pad), min(img.shape[1], x1 + pad)
            cropped = img[y0:y1, x0:x1].copy()

            # Make white background transparent
            white_thresh = 250
            is_white = np.all(cropped[:, :, :3] >= white_thresh, axis=2)
            cropped[is_white, 3] = 0

            cv2.imwrite(save_path, cropped)
            print(f"Saved: {save_path} ({cropped.shape[1]}x{cropped.shape[0]})")

    print("Keys: S=save PNG, W=wireframe, Z=reset view, A=axis, G=grid, Q=quit")
    SAMViewer(scene, background=[255, 255, 255, 255], resolution=[1280, 960], caption="SAM AUV")


if __name__ == "__main__":
    main()
