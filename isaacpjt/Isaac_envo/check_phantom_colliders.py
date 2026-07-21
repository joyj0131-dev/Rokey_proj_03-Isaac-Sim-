#!/usr/bin/env python3
"""팀원 깊이캠 에셋의 /colliders 라이브러리 스코프가 유령 충돌체를 만드는지 검증한다.

이 에셋은 /visuals, /colliders, /meshes 를 최상위 형제 스코프로 두고 로봇 프림이
내부 참조로 가져다 쓴다. 그런데 그 라이브러리 스코프 자체도 active=True 로 남아 있고,
안에 CollisionAPI 프림이 33개 있으며 전부 월드 원점(0,0,0)에 겹쳐 있다.
visible=invisible 은 렌더링만 끄지 물리는 안 끈다.

원점은 주차장 통로 한복판이므로, PhysX가 저것들을 정적 충돌체로 잡으면
로봇이 통로를 못 지나간다.

로봇을 원점에서 멀리(x=10) 놓고 원점 주변만 쿼리해서 구분한다.

실행:
    python3 check_phantom_colliders.py            # 합친 에셋
    python3 check_phantom_colliders.py --source   # 팀원 원본으로도 확인
"""

import json
import os
import sys
from pathlib import Path

WORK_DIR = Path(__file__).resolve().parent
REPO_ROOT = WORK_DIR.parent.parent
ROBOT_PKG = WORK_DIR.parent / "hwia_parking_robot_final_caster_package"

MERGED_USD = ROBOT_PKG / "hwia_depth_cam_mecha_roller.usd"
SOURCE_USD = REPO_ROOT / "hwia_parking_robot_final_caster_camera_mesh_depth_cam.usd"
REPORT = WORK_DIR / "phantom_collider_report.json"
ISAAC_PYTHON = Path(
    "/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh"
)

ROBOT_X = 10.0        # 로봇을 원점에서 멀리 둔다
PROBE_RADIUS = 0.5    # 원점 주변 이 반경 안을 훑는다


def _restart_with_isaac_python():
    if os.environ.get("CARB_APP_PATH"):
        return
    if not ISAAC_PYTHON.is_file():
        raise FileNotFoundError(f"Isaac python.sh를 찾을 수 없습니다: {ISAAC_PYTHON}")
    os.execv(
        str(ISAAC_PYTHON),
        [str(ISAAC_PYTHON), str(Path(__file__).resolve()), *sys.argv[1:]],
    )


def main():
    _restart_with_isaac_python()
    use_source = "--source" in sys.argv[1:]
    robot_usd = SOURCE_USD if use_source else MERGED_USD

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})
    try:
        import carb
        import omni.physx
        import omni.timeline
        from isaacsim.core.api import World
        from pxr import Gf, Usd, UsdGeom, UsdPhysics

        stage = Usd.Stage.CreateInMemory()
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
        UsdGeom.SetStageMetersPerUnit(stage, 1.0)
        world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()
        stage.SetDefaultPrim(world_prim)

        scene = UsdPhysics.Scene.Define(stage, "/World/PhysicsScene")
        scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, -1.0, 0.0))
        scene.CreateGravityMagnitudeAttr(9.81)

        # 평평한 바닥 (y=0 상면)
        ground = UsdGeom.Cube.Define(stage, "/World/Ground")
        ground.CreateSizeAttr(1.0)
        gx = UsdGeom.Xformable(ground)
        gx.AddTranslateOp().Set(Gf.Vec3d(0.0, -0.5, 0.0))
        gx.AddScaleOp().Set(Gf.Vec3f(60.0, 1.0, 60.0))
        UsdPhysics.CollisionAPI.Apply(ground.GetPrim())

        # 로봇을 원점에서 멀리 배치. Z-up 로봇을 Y-up 월드로 돌린다.
        robot = stage.DefinePrim("/World/Robot", "Xform")
        robot.GetReferences().AddReference(str(robot_usd))
        rx = UsdGeom.Xformable(robot)
        rx.ClearXformOpOrder()
        rx.AddTranslateOp().Set(Gf.Vec3d(ROBOT_X, 0.0, 0.0))
        rx.AddRotateXOp().Set(-90.0)

        tmp = WORK_DIR / "_phantom_probe.usd"
        stage.GetRootLayer().Export(str(tmp))

        import omni.usd

        ctx = omni.usd.get_context()
        ctx.open_stage(str(tmp))
        for _ in range(30):
            app.update()

        world = World(stage_units_in_meters=1.0, set_defaults=False)
        omni.timeline.get_timeline_interface().play()
        world.reset()
        for _ in range(60):
            world.step(render=False)

        query = omni.physx.get_physx_scene_query_interface()

        # 1) 원점 위에서 아래로 레이캐스트. 바닥(y=0)보다 위에서 뭔가 맞으면 유령이다.
        hit = query.raycast_closest(
            carb.Float3(0.0, 2.0, 0.0), carb.Float3(0.0, -1.0, 0.0), 5.0
        )
        ray = {"hit": bool(hit and hit.get("hit"))}
        if ray["hit"]:
            ray["collision"] = str(hit.get("collision", ""))
            ray["distance"] = float(hit.get("distance", -1))
            ray["y"] = round(2.0 - ray["distance"], 4)

        # 2) 원점 주변 구 오버랩. 바닥 말고 뭐가 더 있는지 전부 센다.
        found = []

        def _report(overlap):
            found.append(str(overlap.rigid_body) or str(overlap.collision))
            return True

        num = query.overlap_sphere(
            PROBE_RADIUS, carb.Float3(0.0, 0.1, 0.0), _report, False
        )

        phantom = [p for p in found if "Ground" not in p and "Robot" not in p]
        result = {
            "robot_usd": str(robot_usd),
            "robot_placed_at_x": ROBOT_X,
            "raycast_down_at_origin": ray,
            "overlap_sphere_count": int(num),
            "overlap_paths": found,
            "phantom_paths": phantom,
            "phantom_found": bool(phantom) or (
                ray["hit"] and ray.get("y", 0.0) > 0.01
            ),
        }
        REPORT.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")

        print(f"[phantom] 에셋: {robot_usd.name}", flush=True)
        print(f"[phantom] 로봇 배치 x={ROBOT_X} (원점에서 격리)", flush=True)
        print(f"[phantom] 원점 하향 레이캐스트: {ray}", flush=True)
        print(f"[phantom] 원점 반경 {PROBE_RADIUS} m 오버랩: {num}개", flush=True)
        for p in found:
            print(f"            {p}", flush=True)
        print(f"PHANTOM_FOUND={result['phantom_found']}", flush=True)
        tmp.unlink(missing_ok=True)
    finally:
        app.close()


if __name__ == "__main__":
    main()
