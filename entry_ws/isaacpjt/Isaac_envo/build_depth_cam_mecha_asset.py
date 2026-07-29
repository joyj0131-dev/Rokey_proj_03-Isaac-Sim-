#!/usr/bin/env python3
"""깊이 카메라 로봇 에셋에 메카넘 롤러를 얹어 합친 에셋을 만든다.

배경: 팀원이 만든 깊이 카메라 에셋
  hwia_parking_robot_final_caster_camera_mesh_depth_cam.usd
은 **베이스 에셋 위에** 카메라를 붙인 것이라 메카넘 롤러가 없다(roller_* 0개).
그대로 쓰면 횡이동이 불가능해 통로 운반(최대 30 m)이 성립하지 않고,
지금까지의 물리 검증(리프트/운반)도 무효가 된다.

원본은 수정하지 않고, 참조 + override 로 롤러 40개와 조인트를 얹은 별도 에셋을 만든다.
롤러 authoring 은 mecanum_drive.add_mecanum_rollers 를 그대로 재사용한다
(build_mecha_roller_asset.py 가 워크스페이스에 없으므로 그것을 재구현하지 않는다).

주의 — 계층 구조가 기존 에셋과 다르다:
    베이스/메카넘 : <root>/base_link/wheel_fl , <root>/base_link/joints
    깊이캠        : <root>/wheel_fl           , <root>/joints      (한 단계 얕음)
그래서 robot_wrap / robot_joints 경로를 자동 탐지한다.

실행 (GPU/SimulationApp 불필요):
    python3 build_depth_cam_mecha_asset.py
"""

import os
import subprocess
import sys
from pathlib import Path

WORK_DIR = Path(__file__).resolve().parent
REPO_ROOT = WORK_DIR.parent.parent
ROBOT_PKG = WORK_DIR.parent / "hwia_parking_robot_final_caster_package"

SOURCE_USD = REPO_ROOT / "hwia_parking_robot_final_caster_camera_mesh_depth_cam.usd"
OUTPUT_USD = ROBOT_PKG / "hwia_depth_cam_mecha_roller.usd"

ISAAC_RELEASE = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release")

# 기존 메카넘 에셋의 RollerMaterial 과 동일한 값 (실측 확인).
ROLLER_FRICTION_STATIC = 1.10
ROLLER_FRICTION_DYNAMIC = 0.90
ROLLER_RESTITUTION = 0.0

WHEELS = ("wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr")


def _reexec():
    try:
        import pxr  # noqa: F401

        return
    except ModuleNotFoundError:
        pass
    if os.environ.get("_DEPTHCAM_MECHA_REEXEC"):
        raise RuntimeError("재실행 후에도 pxr을 import하지 못했습니다.")
    libs = next(
        (c for c in sorted((ISAAC_RELEASE / "extscache").glob("omni.usd.libs-*"))
         if (c / "pxr").exists()),
        None,
    )
    if libs is None:
        raise FileNotFoundError("omni.usd.libs(pxr)를 찾지 못했습니다.")
    env = dict(
        os.environ,
        _DEPTHCAM_MECHA_REEXEC="1",
        PYTHONPATH=os.pathsep.join(
            [str(libs), str(WORK_DIR), os.environ.get("PYTHONPATH", "")]
        ).strip(os.pathsep),
        LD_LIBRARY_PATH=os.pathsep.join(
            [str(libs / "bin"), os.environ.get("LD_LIBRARY_PATH", "")]
        ).strip(os.pathsep),
    )
    raise SystemExit(
        subprocess.call(
            [str(ISAAC_RELEASE / "python.sh"), str(Path(__file__).resolve()), *sys.argv[1:]],
            env=env,
            cwd=str(ISAAC_RELEASE),
        )
    )


def _locate(stage, root_path):
    """휠과 joints 스코프가 어느 깊이에 있는지 찾는다.

    에셋마다 계층이 달라 경로를 박아두면 안 된다.
    """
    wheel = None
    for prim in stage.Traverse():
        if prim.GetName() == WHEELS[0]:
            wheel = prim
            break
    if wheel is None:
        raise RuntimeError(f"{WHEELS[0]} 프림을 찾지 못했습니다.")
    wrap = wheel.GetParent().GetPath()

    joints = None
    for prim in stage.Traverse():
        if prim.GetName() == "joints":
            joints = prim.GetPath()
            break
    if joints is None:
        raise RuntimeError("joints 스코프를 찾지 못했습니다.")
    return str(wrap), str(joints)


def build():
    from pxr import Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

    import mecanum_drive as MD

    if not SOURCE_USD.is_file():
        raise FileNotFoundError(f"깊이 카메라 에셋이 없습니다: {SOURCE_USD}")
    if OUTPUT_USD.exists():
        OUTPUT_USD.unlink()

    src = Usd.Stage.Open(str(SOURCE_USD))
    src_default = src.GetDefaultPrim()
    if not src_default:
        raise RuntimeError("원본에 defaultPrim이 없습니다.")
    root_name = src_default.GetName()

    stage = Usd.Stage.CreateNew(str(OUTPUT_USD))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.GetStageUpAxis(src))
    UsdGeom.SetStageMetersPerUnit(stage, UsdGeom.GetStageMetersPerUnit(src))

    # 서브레이어로 깐다. reference 로 defaultPrim 만 가져오면 형제 스코프
    # (/visuals, /colliders, /meshes)가 빠져 충돌체·메시가 통째로 사라진다.
    rel = os.path.relpath(SOURCE_USD, OUTPUT_USD.parent)
    stage.GetRootLayer().subLayerPaths.append(rel)

    root = stage.GetPrimAtPath(f"/{root_name}")
    if not root or not root.IsValid():
        raise RuntimeError(f"서브레이어에서 /{root_name} 을 찾지 못했습니다.")
    stage.SetDefaultPrim(root)

    # 이 에셋은 인스턴싱을 쓴다. 충돌체가 프로토타입 안에 있어 인스턴스를 통해서는
    # 수정할 수 없다(허브 충돌 비활성화가 조용히 실패한다). 필요한 곳만 인스턴싱을 푼다.
    uninstanced = 0
    for prim in stage.Traverse():
        if prim.IsInstance():
            prim.SetInstanceable(False)
            uninstanced += 1

    wrap, joints = _locate(stage, f"/{root_name}")

    # 롤러 전용 물리 재질. 기존 메카넘 에셋과 같은 값.
    grip = UsdShade.Material.Define(stage, f"/{root_name}/RollerMaterial")
    api = UsdPhysics.MaterialAPI.Apply(grip.GetPrim())
    api.CreateStaticFrictionAttr(ROLLER_FRICTION_STATIC)
    api.CreateDynamicFrictionAttr(ROLLER_FRICTION_DYNAMIC)
    api.CreateRestitutionAttr(ROLLER_RESTITUTION)

    MD.add_mecanum_rollers(stage, wrap, joints, grip_material=grip)

    # add_mecanum_rollers 는 허브 충돌을 collisionEnabled=False 로 '끄기만' 한다.
    # 기존 메카넘 에셋은 허브 충돌체가 아예 없는 상태였고(flatten 과정에서 제거),
    # 끈 채로 남겨두면 구동 시 로봇이 공중으로 튀어오르는 현상이 관측됐다
    # (전진 명령에 y=+2.48 m, 휠 각속도가 명령값 그대로 = 접지 안 됨).
    # 그래서 허브 충돌체 프림 자체를 비활성화해 기존 에셋과 조건을 맞춘다.
    disabled = 0
    for prim in stage.Traverse():
        if prim.GetName() not in WHEELS:
            continue
        for sub in Usd.PrimRange(prim):
            if sub.HasAPI(UsdPhysics.CollisionAPI):
                sub.SetActive(False)
                disabled += 1

    # 캐스터 휠 충돌 해제 — 이게 없으면 구동 중 로봇이 발사된다.
    #
    # 이 에셋의 베이스는 캐스터 구동 설계라 수동 캐스터 휠 4개가 달려 있다.
    # 그 충돌체는 월드 y=0.000~0.056 으로 지면에 정확히 닿는데, 메카넘 롤러는
    # y=-0.011 로 지면보다 낮게 오서링돼 있다. 둘이 동시에 접지하면 PhysX가
    # 서로 모순되는 접촉을 풀면서 에너지를 쌓고, 실측으로 구동 82스텝에서 14 cm
    # 튄 뒤 108스텝에 vy≈12 m/s 임펄스로 발사돼 완전히 탄도 운동을 했다
    # (140스텝 후 y=+4.53 m). 캐스터 충돌만 꺼도 y가 ±0.0006 m 안에서 유지되고
    # 횡드리프트가 0.19 m -> 0.000 m 로 사라진다.
    #
    # 기존 메카넘 에셋(hwia_parking_robot_final_caster_mecha_roller.usd)은 애초에
    # 롤러 외 충돌체가 0개라 이 문제가 없었다. 여기서는 캐스터만 끄고 섀시/암
    # 충돌체는 남긴다 — 차량을 들어올릴 때 필요하다.
    casters = 0
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            continue
        if "caster_" not in str(prim.GetPath()):
            continue
        UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr(False)
        casters += 1

    stage.GetRootLayer().Save()
    return root_name, wrap, joints, rel, uninstanced, disabled, casters


def verify():
    from pxr import Usd, UsdPhysics

    stage = Usd.Stage.Open(str(OUTPUT_USD))
    rollers = [p for p in stage.Traverse()
               if p.GetName().startswith("roller_") and p.GetTypeName() == "Capsule"]
    joints = [p for p in stage.Traverse()
              if p.GetName().startswith("roller_")
              and p.GetTypeName() == "PhysicsRevoluteJoint"]
    cams = [p for p in stage.Traverse()
            if p.GetTypeName() == "Camera" and "OmniverseKit" not in str(p.GetPath())]

    # 허브 충돌이 실제로 꺼졌는지 — 서브트리 전체를 훑는다(충돌체는 여러 단계 아래에 있다).
    hubs_off = []
    for prim in stage.Traverse():
        if prim.GetName() not in WHEELS:
            continue
        states = [
            UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get()
            for p in Usd.PrimRange(prim)
            if p.HasAPI(UsdPhysics.CollisionAPI)
        ]
        hubs_off.append((prim.GetName(), states))

    # 캐스터 충돌이 하나라도 살아 있으면 구동 중 발사된다. 개수가 아니라
    # "켜진 게 0개"를 확인해야 한다.
    casters_on = [
        str(p.GetPath())
        for p in stage.Traverse()
        if p.HasAPI(UsdPhysics.CollisionAPI)
        and "caster_" in str(p.GetPath())
        and UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get() is not False
    ]

    return {
        "rollers": len(rollers),
        "roller_joints": len(joints),
        "cameras": len(cams),
        "hub_collision": hubs_off,
        "casters_on": casters_on,
    }


def main():
    _reexec()
    root_name, wrap, joints, rel, uninstanced, disabled, casters = build()
    info = verify()

    print(f"[mecha] 원본(무수정): {SOURCE_USD.name}")
    print(f"[mecha] 출력:         {OUTPUT_USD}")
    print(f"[mecha] 서브레이어:   {rel}")
    print(f"[mecha] 허브 충돌체 비활성: {disabled}개 (끄기만 하면 구동 시 튀어오름)")
    print(f"[mecha] 인스턴싱 해제: {uninstanced}개 (프로토타입 안 충돌체를 수정하려면 필수)")
    print(f"[mecha] defaultPrim = /{root_name}")
    print(f"[mecha] 휠 스코프    = {wrap}")
    print(f"[mecha] 조인트 스코프 = {joints}")
    print(f"[mecha] 롤러 {info['rollers']}개 / 롤러조인트 {info['roller_joints']}개 "
          f"/ 카메라 {info['cameras']}대")
    for name, states in info["hub_collision"]:
        print(f"[mecha]   허브 {name} 충돌체 {len(states)}개 enabled={states} (False 여야 롤러만 접지)")
    print(f"[mecha] 캐스터 충돌 해제: {casters}개 / 아직 켜진 캐스터: {len(info['casters_on'])}개 "
          f"(0이어야 함 — 남으면 구동 중 로봇이 발사됨)")
    for p in info["casters_on"]:
        print(f"[mecha]   !! 캐스터 충돌 살아있음: {p}")

    ok = (info["rollers"] == 40 and info["roller_joints"] == 40
          and info["cameras"] >= 4 and not info["casters_on"])
    print(f"BUILD_OK={ok}")
    if not ok:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
