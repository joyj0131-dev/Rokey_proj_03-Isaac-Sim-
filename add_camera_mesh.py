#!/usr/bin/env python3
"""Replace the plain 'mint block' camera placeholders in the URDF with a
compound, actually camera-shaped visual (housing + sensor strip + 3 lens
barrels, RealSense-D435-like layout), directly in the robot's source URDF.

This only touches cam_side_left_link / cam_side_right_link. cam_front_link
and cam_qr_down_link are left as-is (not requested).

Footprint is kept identical to the original box (0.034 x 0.040 x 0.025 m)
so nothing else on the robot (arm clearance, existing joint origins) needs
to change -- the new geometry is added *around* that same footprint, not
instead of it.

NOTE ON LENS-FACING DIRECTION:
The lens barrels are placed on the link's local +X face. Whether that
ends up pointing outward/inward once the existing joint rpy
(cam_side_left: +90 deg about X, cam_side_right: -90 deg about X) is
applied could not be verified without opening this in Isaac Sim. After
importing, look at the robot from the front in the viewport; if the
lenses appear to face into the chassis instead of outward, flip the sign
of LENS_FACE_SIGN below and rerun.

Usage:
  python3 add_camera_mesh.py <input.urdf> <output.urdf>
"""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

LENS_FACE_SIGN = 1  # flip to -1 if the lenses face the wrong way after import

NEW_MATERIALS = '''  <material name="cam_case_black"><color rgba="0.05 0.05 0.06 1"/></material>
  <material name="cam_silver"><color rgba="0.74 0.75 0.78 1"/></material>
  <material name="lens_black"><color rgba="0.02 0.02 0.03 1"/></material>
'''

HOUSING_W = 0.034   # local X (depth, front-to-back)
HOUSING_H = 0.040   # local Y (height in link frame)
HOUSING_D = 0.025   # local Z (width in link frame) -- unchanged footprint

STRIP_X = HOUSING_W / 2.0 - 0.002        # sits just proud of the front face
STRIP_SIZE = (0.006, HOUSING_H * 0.85, 0.010)

LENS_RADIUS = 0.0035
LENS_LEN = 0.007
LENS_X = STRIP_X + LENS_LEN / 2.0 * LENS_FACE_SIGN
LENS_Y_OFFSETS = (-0.012, 0.0, 0.012)  # three lenses: IR-left, RGB-center, IR-right


def _camera_visual_xml(link_prefix: str) -> str:
    lens_visuals = []
    for i, y in enumerate(LENS_Y_OFFSETS):
        lens_visuals.append(f'''    <visual name="{link_prefix}_lens_{i}">
      <origin xyz="{LENS_X:.4f} {y:.3f} 0" rpy="0 1.57079632679 0"/>
      <geometry><cylinder radius="{LENS_RADIUS}" length="{LENS_LEN}"/></geometry>
      <material name="lens_black"/>
    </visual>''')
    lens_block = "\n".join(lens_visuals)

    return f'''    <visual name="{link_prefix}_housing">
      <geometry><box size="{HOUSING_W} {HOUSING_H} {HOUSING_D}"/></geometry>
      <material name="cam_case_black"/>
    </visual>
    <visual name="{link_prefix}_sensor_strip">
      <origin xyz="{STRIP_X:.4f} 0 0" rpy="0 0 0"/>
      <geometry><box size="{STRIP_SIZE[0]} {STRIP_SIZE[1]} {STRIP_SIZE[2]}"/></geometry>
      <material name="cam_silver"/>
    </visual>
{lens_block}'''


def transform(text: str) -> str:
    # 1) add the new materials next to the existing sensor_blue definition
    marker = '  <material name="sensor_blue"><color rgba="0.08 0.60 0.92 1"/></material>\n'
    if marker not in text:
        raise RuntimeError("sensor_blue material line not found; URDF format may have changed")
    text = text.replace(marker, marker + NEW_MATERIALS, 1)

    # 2) cam_side_left_link
    old_left = ('  <link name="cam_side_left_link">\n'
                '    <visual><geometry><box size="0.034 0.040 0.025"/></geometry>'
                '<material name="sensor_blue"/></visual>\n'
                '  </link>')
    if old_left not in text:
        raise RuntimeError("cam_side_left_link block not found / already modified")
    new_left = ('  <link name="cam_side_left_link">\n'
                f'{_camera_visual_xml("cam_side_left")}\n'
                '  </link>')
    text = text.replace(old_left, new_left, 1)

    # 3) cam_side_right_link
    old_right = ('  <link name="cam_side_right_link">\n'
                 '    <visual><geometry><box size="0.034 0.040 0.025"/></geometry>'
                 '<material name="sensor_blue"/></visual>\n'
                 '  </link>')
    if old_right not in text:
        raise RuntimeError("cam_side_right_link block not found / already modified")
    new_right = ('  <link name="cam_side_right_link">\n'
                 f'{_camera_visual_xml("cam_side_right")}\n'
                 '  </link>')
    text = text.replace(old_right, new_right, 1)

    return text


def main():
    if len(sys.argv) != 3:
        print("Usage: add_camera_mesh.py <input.urdf> <output.urdf>")
        sys.exit(1)

    src = Path(sys.argv[1])
    out = Path(sys.argv[2])

    text = src.read_text(encoding="utf-8")
    new_text = transform(text)
    out.write_text(new_text, encoding="utf-8")

    # Validate: parse XML, check the expected visual names exist.
    root = ET.parse(out).getroot()
    left_link = next(l for l in root.findall("link") if l.attrib.get("name") == "cam_side_left_link")
    right_link = next(l for l in root.findall("link") if l.attrib.get("name") == "cam_side_right_link")
    left_visuals = left_link.findall("visual")
    right_visuals = right_link.findall("visual")

    print(f"XML parse: PASS")
    print(f"cam_side_left_link visuals: {len(left_visuals)} (expected 5: housing+strip+3 lenses)")
    print(f"cam_side_right_link visuals: {len(right_visuals)} (expected 5)")
    assert len(left_visuals) == 5
    assert len(right_visuals) == 5
    print(f"Wrote {out}")


if __name__ == "__main__":
    main()
