from pathlib import Path
import xml.etree.ElementTree as ET
import zipfile

src = Path('/mnt/data/hwia_parking_robot_bearing_roller.urdf')
out = Path('/mnt/data/hwia_parking_robot_final_caster.urdf')
readme = Path('/mnt/data/hwia_parking_robot_final_caster_README.md')
zip_path = Path('/mnt/data/hwia_parking_robot_final_caster_package.zip')

text = src.read_text(encoding='utf-8')
text = text.replace('<robot name="hwia_parking_robot_bearing_roller">', '<robot name="hwia_parking_robot_final_caster">', 1)
text = text.replace(
    'Each arm has a freely rotating bearing roller at its tire-contact end.\n    As the front/rear arm pair closes, the rollers spin against the tire surface,\n    reducing sliding friction while rolling under the lower tire to create passive lift.',
    'Each arm has a freely rotating bearing roller at its tire-contact end.\n    Each arm tip also has a passive swivel caster that supports the arm on the floor.\n    As the front/rear arm pair closes, the rollers spin against the tire surface while\n    the casters carry vertical load into the floor, reducing bending load on the arm pivots.'
)

caster_section = r'''
  <!-- ============================================================
       ARM-TIP PASSIVE SWIVEL CASTERS
       Each swing arm has one chair-style caster at its extreme tip.
       The caster has two passive DOFs:
         1) caster_*_swivel_joint : continuous rotation about local Z
         2) caster_*_wheel_joint  : continuous wheel rotation about local Y
       The caster wheel radius/height are chosen so the wheel touches the ground
       when the robot base and arms are at their nominal height.
       These casters do NOT lift the tire. They support the arm tip and transfer
       vertical vehicle load to the floor while the bearing roller handles tire contact.
       ============================================================ -->

  <link name="caster_left_front_fork">
    <inertial>
      <origin xyz="0.012 0 -0.012" rpy="0 0 0"/>
      <mass value="0.45"/>
      <inertia ixx="0.00035" ixy="0" ixz="0" iyy="0.00035" iyz="0" izz="0.00020"/>
    </inertial>
    <visual name="swivel_stem">
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><cylinder radius="0.012" length="0.030"/></geometry>
      <material name="body_dark"/>
    </visual>
    <visual name="fork_bracket">
      <origin xyz="0.012 0 -0.018" rpy="0 0 0"/>
      <geometry><box size="0.050 0.038 0.018"/></geometry>
      <material name="arm_light"/>
    </visual>
  </link>
  <joint name="caster_left_front_swivel_joint" type="continuous">
    <parent link="arm_left_front"/>
    <child link="caster_left_front_fork"/>
    <origin xyz="0.50 0 -0.015" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <dynamics damping="0.04" friction="0.02"/>
  </joint>

  <link name="caster_left_front_wheel">
    <inertial>
      <mass value="0.35"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <inertia ixx="0.00012" ixy="0" ixz="0" iyy="0.00008" iyz="0" izz="0.00012"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="1.57079632679 0 0"/>
      <geometry><cylinder radius="0.028" length="0.022"/></geometry>
      <material name="wheel_black"/>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="1.57079632679 0 0"/>
      <geometry><cylinder radius="0.028" length="0.022"/></geometry>
    </collision>
  </link>
  <joint name="caster_left_front_wheel_joint" type="continuous">
    <parent link="caster_left_front_fork"/>
    <child link="caster_left_front_wheel"/>
    <origin xyz="0.025 0 -0.032" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <dynamics damping="0.015" friction="0.005"/>
  </joint>

  <link name="caster_left_rear_fork">
    <inertial>
      <origin xyz="0.012 0 -0.012" rpy="0 0 0"/>
      <mass value="0.45"/>
      <inertia ixx="0.00035" ixy="0" ixz="0" iyy="0.00035" iyz="0" izz="0.00020"/>
    </inertial>
    <visual name="swivel_stem"><geometry><cylinder radius="0.012" length="0.030"/></geometry><material name="body_dark"/></visual>
    <visual name="fork_bracket"><origin xyz="0.012 0 -0.018"/><geometry><box size="0.050 0.038 0.018"/></geometry><material name="arm_light"/></visual>
  </link>
  <joint name="caster_left_rear_swivel_joint" type="continuous">
    <parent link="arm_left_rear"/><child link="caster_left_rear_fork"/>
    <origin xyz="0.50 0 -0.015" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <dynamics damping="0.04" friction="0.02"/>
  </joint>
  <link name="caster_left_rear_wheel">
    <inertial><mass value="0.35"/><origin xyz="0 0 0"/><inertia ixx="0.00012" ixy="0" ixz="0" iyy="0.00008" iyz="0" izz="0.00012"/></inertial>
    <visual><origin rpy="1.57079632679 0 0"/><geometry><cylinder radius="0.028" length="0.022"/></geometry><material name="wheel_black"/></visual>
    <collision><origin rpy="1.57079632679 0 0"/><geometry><cylinder radius="0.028" length="0.022"/></geometry></collision>
  </link>
  <joint name="caster_left_rear_wheel_joint" type="continuous">
    <parent link="caster_left_rear_fork"/><child link="caster_left_rear_wheel"/>
    <origin xyz="0.025 0 -0.032"/><axis xyz="0 1 0"/>
    <dynamics damping="0.015" friction="0.005"/>
  </joint>

  <link name="caster_right_front_fork">
    <inertial>
      <origin xyz="0.012 0 -0.012" rpy="0 0 0"/>
      <mass value="0.45"/>
      <inertia ixx="0.00035" ixy="0" ixz="0" iyy="0.00035" iyz="0" izz="0.00020"/>
    </inertial>
    <visual name="swivel_stem"><geometry><cylinder radius="0.012" length="0.030"/></geometry><material name="body_dark"/></visual>
    <visual name="fork_bracket"><origin xyz="0.012 0 -0.018"/><geometry><box size="0.050 0.038 0.018"/></geometry><material name="arm_light"/></visual>
  </link>
  <joint name="caster_right_front_swivel_joint" type="continuous">
    <parent link="arm_right_front"/><child link="caster_right_front_fork"/>
    <origin xyz="0.50 0 -0.015" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <dynamics damping="0.04" friction="0.02"/>
  </joint>
  <link name="caster_right_front_wheel">
    <inertial><mass value="0.35"/><origin xyz="0 0 0"/><inertia ixx="0.00012" ixy="0" ixz="0" iyy="0.00008" iyz="0" izz="0.00012"/></inertial>
    <visual><origin rpy="1.57079632679 0 0"/><geometry><cylinder radius="0.028" length="0.022"/></geometry><material name="wheel_black"/></visual>
    <collision><origin rpy="1.57079632679 0 0"/><geometry><cylinder radius="0.028" length="0.022"/></geometry></collision>
  </link>
  <joint name="caster_right_front_wheel_joint" type="continuous">
    <parent link="caster_right_front_fork"/><child link="caster_right_front_wheel"/>
    <origin xyz="0.025 0 -0.032"/><axis xyz="0 1 0"/>
    <dynamics damping="0.015" friction="0.005"/>
  </joint>

  <link name="caster_right_rear_fork">
    <inertial>
      <origin xyz="0.012 0 -0.012" rpy="0 0 0"/>
      <mass value="0.45"/>
      <inertia ixx="0.00035" ixy="0" ixz="0" iyy="0.00035" iyz="0" izz="0.00020"/>
    </inertial>
    <visual name="swivel_stem"><geometry><cylinder radius="0.012" length="0.030"/></geometry><material name="body_dark"/></visual>
    <visual name="fork_bracket"><origin xyz="0.012 0 -0.018"/><geometry><box size="0.050 0.038 0.018"/></geometry><material name="arm_light"/></visual>
  </link>
  <joint name="caster_right_rear_swivel_joint" type="continuous">
    <parent link="arm_right_rear"/><child link="caster_right_rear_fork"/>
    <origin xyz="0.50 0 -0.015" rpy="0 0 0"/><axis xyz="0 0 1"/>
    <dynamics damping="0.04" friction="0.02"/>
  </joint>
  <link name="caster_right_rear_wheel">
    <inertial><mass value="0.35"/><origin xyz="0 0 0"/><inertia ixx="0.00012" ixy="0" ixz="0" iyy="0.00008" iyz="0" izz="0.00012"/></inertial>
    <visual><origin rpy="1.57079632679 0 0"/><geometry><cylinder radius="0.028" length="0.022"/></geometry><material name="wheel_black"/></visual>
    <collision><origin rpy="1.57079632679 0 0"/><geometry><cylinder radius="0.028" length="0.022"/></geometry></collision>
  </link>
  <joint name="caster_right_rear_wheel_joint" type="continuous">
    <parent link="caster_right_rear_fork"/><child link="caster_right_rear_wheel"/>
    <origin xyz="0.025 0 -0.032"/><axis xyz="0 1 0"/>
    <dynamics damping="0.015" friction="0.005"/>
  </joint>

'''

marker = '  <!-- ================= Sensors ================= -->'
if marker not in text:
    raise RuntimeError('sensor marker not found')
text = text.replace(marker, caster_section + marker, 1)

text = text.replace(
    '6) bearing_roller_*_joint -> PASSIVE continuous joints. Do NOT add position/velocity drives.\n      7) For actual tire lift,',
    '6) bearing_roller_*_joint -> PASSIVE continuous joints. Do NOT add position/velocity drives.\n      7) caster_*_swivel_joint and caster_*_wheel_joint -> PASSIVE continuous joints.\n         Do NOT add position/velocity drives; keep damping/friction low so each caster self-aligns and rolls.\n      8) For actual tire lift,'
)
text = text.replace(
    'In Isaac Sim, assign a low-friction Physics Material to the bearing roller collisions\n         and tune arm drive effort, tire friction, roller radius, and arm pivot spacing.',
    'In Isaac Sim, tune the bearing-roller/tire contact material, caster-wheel/floor friction,\n         arm drive effort, roller radius, caster height, and arm pivot spacing. The caster wheels\n         should stay in light contact with the floor so the arm tips can carry vertical load.'
)

out.write_text(text, encoding='utf-8')

# XML parse validation and simple structure checks.
root = ET.parse(out).getroot()
links = root.findall('link')
joints = root.findall('joint')
link_names = {x.attrib['name'] for x in links}
joint_names = {x.attrib['name'] for x in joints}
expected = {
    'caster_left_front_fork','caster_left_front_wheel',
    'caster_left_rear_fork','caster_left_rear_wheel',
    'caster_right_front_fork','caster_right_front_wheel',
    'caster_right_rear_fork','caster_right_rear_wheel',
}
assert expected.issubset(link_names)
assert sum(1 for j in joint_names if j.startswith('caster_')) == 8

readme.write_text(f'''# Hyundai WIA-style Parking Robot — Bearing Roller + Arm-tip Caster Revision

최종 수정 버전입니다.

## 핵심 구조
- 4개의 swing arm은 `base_link`에 직접 연결됩니다.
- 각 팔은 평소 0 rad에서 본체와 나란히 접혀 있습니다.
- 차량 바퀴를 잡을 때 4개 팔이 Z축 revolute joint로 펼쳐집니다.
- 각 팔의 타이어 접촉부에는 자유회전 `bearing_roller_*`가 있어 타이어 표면을 따라 굴러갑니다.
- 각 팔의 맨 끝에는 의자 바퀴 형태의 **passive swivel caster**가 추가되었습니다.

## 캐스터 구조
각 팔마다 캐스터 1개가 있고, 캐스터마다 2개 passive joint가 있습니다.

1. `caster_*_swivel_joint`
   - 팔 끝에 연결
   - Z축 `continuous`
   - 캐스터가 진행 방향에 맞춰 자유롭게 방향을 돌림

2. `caster_*_wheel_joint`
   - 캐스터 포크와 작은 지지 바퀴 연결
   - Y축 `continuous`
   - 지면 위에서 수동으로 굴러감

## 목적
캐스터는 차량 타이어를 들어 올리는 장치가 아닙니다. 타이어는 기존의 bearing roller와 swing arm의 파지 동작으로 올라갑니다. 캐스터는 팔 끝을 지면에서 지지해 차량 하중 일부를 바닥으로 전달하고, 팔 자체와 팔 pivot에 걸리는 굽힘 모멘트를 줄이기 위한 보조 지지 바퀴입니다.

## 기본 치수
- 캐스터 휠 반지름: 0.028 m
- 캐스터 휠 폭: 0.022 m
- 팔 pivot 기준 캐스터 swivel 위치: local X = 0.50 m
- nominal wheel center height: 약 0.028 m

## Isaac Sim 권장 설정
- `caster_*_swivel_joint`: drive 없음, 낮은 damping/friction
- `caster_*_wheel_joint`: drive 없음, 낮은 damping/friction
- caster wheel ↔ floor: 미끄러지지 않으면서 잘 구를 수 있도록 중간 정도 마찰부터 튜닝
- bearing roller ↔ tire: 롤러가 타이어를 따라 회전하면서 하단으로 진입하도록 접촉 마찰 튜닝
- 팔을 펼친 상태에서 캐스터 휠이 지면에 아주 약하게 접촉하도록 높이를 맞추는 것이 중요

## 검증
- XML parse: PASS
- Links: {len(links)}
- Joints: {len(joints)}
- Bearing roller passive joints: 4
- Caster swivel passive joints: 4
- Caster wheel passive joints: 4
''', encoding='utf-8')

with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zf:
    for p in [out, readme, Path('/mnt/data/add_tip_casters.py')]:
        zf.write(p, arcname=p.name)

print(out)
print(readme)
print(zip_path)
print('links', len(links), 'joints', len(joints))
