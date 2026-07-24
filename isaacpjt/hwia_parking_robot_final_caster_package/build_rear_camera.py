#!/usr/bin/env python3
"""로봇 에셋에서 cam_front 를 미러한 cam_rear 를 확보하고 cam_qr_down 을 제거해
4-카메라 에셋(front / rear / side_left / side_right)을 만든다.

측면 뎁스캠이 바퀴감지를 담당하므로 cam_rear 는 RGB ArUco 용이다. cam_rear 의
마운트는 base_link 수직축(Z) 180° 회전 미러(뒤 −X 를 향하고 하향각 유지)여야 한다:
카메라 월드 포즈 기준 T_rear = Rz(180°)·T_front.

실측 주의(중요): 이 원본 크레이트는 브리프 가정과 다르다.
  - cam_rear 가 이미 올바른 Rz(180) 미러로 저작되어 있다(카메라 월드 포즈로 검증).
  - cam_qr_down 은 '카메라'가 아니라 빈 링크(cam_qr_down_link, 박스 비주얼만)로만 남아 있다.
  - 루트 레이어에 OmniverseKit 뷰포트 카메라 4개가 저장돼 있어 카메라 프림이 총 8개다.
따라서 저작은: 올바른 cam_rear 는 그대로 유지, cam_qr_down_link(및 관련 조인트) 제거,
OmniverseKit 에디터 카메라 제거 → 파일에 로봇 카메라 정확히 4개만 남긴다.

실행: python.sh build_rear_camera.py --inspect   # 카메라/링크 요약
      python.sh build_rear_camera.py --deep      # 서브트리·월드포즈 심층 출력
      python.sh build_rear_camera.py             # 저작 + 저장 + 검증
"""
import sys
from pathlib import Path

PKG = Path(__file__).resolve().parent
SRC = PKG / "hwia_depth_cam_mecha_roller_lowered.usd"
OUT = PKG / "hwia_4cam_mecha_roller_lowered.usd"
ISAAC_PY = Path("/home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh")

ROBOT = "/hwia_parking_robot_final_caster"          # 기본 프림(로봇 루트)
KIT_CAMS = ("/OmniverseKit_Persp", "/OmniverseKit_Front",
            "/OmniverseKit_Top", "/OmniverseKit_Right")


def _boot():
    from isaacsim import SimulationApp
    return SimulationApp({"headless": True})


def _fmt_m(m):
    """4x4 행렬을 한 줄 문자열로."""
    return " ".join("[" + " ".join(f"{m[r][c]:+.5f}" for c in range(4)) + "]"
                    for r in range(4))


def _cam_pose(stage, cam_prim):
    """카메라 프림의 월드 위치·전방(-Z)·상(+Y) 벡터(하나의 XformCache 기준)."""
    from pxr import UsdGeom, Usd, Gf
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())
    wm = cache.GetLocalToWorldTransform(cam_prim)
    pos = wm.ExtractTranslation()
    fwd = wm.TransformDir(Gf.Vec3d(0, 0, -1)).GetNormalized()
    up = wm.TransformDir(Gf.Vec3d(0, 1, 0)).GetNormalized()
    return pos, fwd, up


# ---------------------------------------------------------------------------
# 인스펙트(구조 확인)
# ---------------------------------------------------------------------------
def inspect(stage):
    """실측 카메라 계층·경로·변환 요약."""
    from pxr import UsdGeom, Usd
    root = stage.GetDefaultPrim()
    print(f"REARCAM_INSPECT_ROOT default_prim={root.GetPath() if root else None}", flush=True)
    cams = [p for p in stage.Traverse() if p.GetTypeName() == "Camera"]
    print(f"REARCAM_INSPECT camera_count={len(cams)}", flush=True)
    for p in cams:
        print(f"  CAMERA {p.GetPath()}", flush=True)
    for link in ("cam_front_link", "cam_qr_down_link", "cam_side_left_link",
                 "cam_side_right_link", "cam_rear_link"):
        for x in stage.Traverse():
            if x.GetName() == link:
                xf = UsdGeom.Xformable(x)
                m = xf.GetLocalTransformation(Usd.TimeCode.Default())
                t = m.ExtractTranslation()
                ops = [op.GetOpName() for op in xf.GetOrderedXformOps()]
                print(f"  LINK {link} path={x.GetPath()} "
                      f"t=({t[0]:+.3f},{t[1]:+.3f},{t[2]:+.3f})", flush=True)
                print(f"  LINK_MATRIX {link} M={_fmt_m(m)}", flush=True)
                print(f"  LINK_XFORMOPS {link} ops={ops}", flush=True)


def deep_inspect(stage):
    """cam_front / cam_rear / cam_qr_down 서브트리 로컬 변환 + 카메라 월드 포즈,
    그리고 OmniverseKit 카메라가 루트/세션 어디에 있는지 판정."""
    from pxr import UsdGeom, Usd

    def find(name):
        for x in stage.Traverse():
            if x.GetName() == name and str(x.GetPath()).startswith(ROBOT + "/"):
                return x
        return None

    for linkname in ("cam_front_link", "cam_rear_link", "cam_qr_down_link"):
        link = find(linkname)
        print(f"DEEP === {linkname} present={link is not None} ===", flush=True)
        if link is None:
            continue
        for p in Usd.PrimRange(link):
            xf = UsdGeom.Xformable(p)
            t = xf.GetLocalTransformation(Usd.TimeCode.Default()).ExtractTranslation()
            ops = [o.GetOpName() for o in xf.GetOrderedXformOps()]
            tag = " [CAMERA]" if p.GetTypeName() == "Camera" else ""
            rel = str(p.GetPath()).replace(ROBOT + "/" + linkname, "")
            print(f"  DEEP_LOCAL {linkname}{rel or '/'} "
                  f"t=({t[0]:+.4f},{t[1]:+.4f},{t[2]:+.4f}) ops={ops}{tag}", flush=True)
        for p in Usd.PrimRange(link):
            if p.GetTypeName() == "Camera":
                pos, fwd, up = _cam_pose(stage, p)
                print(f"  DEEP_CAMWORLD {p.GetName()} "
                      f"pos=({pos[0]:+.4f},{pos[1]:+.4f},{pos[2]:+.4f}) "
                      f"fwd=({fwd[0]:+.4f},{fwd[1]:+.4f},{fwd[2]:+.4f}) "
                      f"up=({up[0]:+.4f},{up[1]:+.4f},{up[2]:+.4f})", flush=True)

    rl, sl = stage.GetRootLayer(), stage.GetSessionLayer()
    for kit in KIT_CAMS:
        print(f"  DEEP_KITCAM {kit} in_root_layer={rl.GetPrimAtPath(kit) is not None} "
              f"in_session_layer={sl.GetPrimAtPath(kit) is not None}", flush=True)


# ---------------------------------------------------------------------------
# 저작
# ---------------------------------------------------------------------------
def _robot_cam_path(stage, linktoken):
    """로봇 루트 아래에서 특정 링크(cam_front_link 등)에 속한 Camera 프림 경로."""
    for p in stage.Traverse():
        if p.GetTypeName() == "Camera" and ("/" + linktoken + "/") in str(p.GetPath()):
            return p
    return None


def _rear_mirror_check(stage, front_cam, rear_cam, tol=1e-3):
    """cam_rear 카메라 월드 포즈가 Rz(180)·cam_front 인지 검증(위치·전방·상 벡터)."""
    from pxr import Gf
    fp, ff, fu = _cam_pose(stage, front_cam)
    rp, rf, ru = _cam_pose(stage, rear_cam)
    # Rz(180): (x,y,z) -> (-x,-y,z)
    ep = Gf.Vec3d(-fp[0], -fp[1], fp[2])
    ef = Gf.Vec3d(-ff[0], -ff[1], ff[2])
    eu = Gf.Vec3d(-fu[0], -fu[1], fu[2])
    pe = (Gf.Vec3d(rp) - ep).GetLength()
    fe = (Gf.Vec3d(rf) - ef).GetLength()
    ue = (Gf.Vec3d(ru) - eu).GetLength()
    ok = pe < tol and fe < tol and ue < tol
    print(f"REARCAM_MIRROR ok={ok} pos_err={pe:.2e} fwd_err={fe:.2e} up_err={ue:.2e}",
          flush=True)
    print(f"  CMP front pos=({fp[0]:+.4f},{fp[1]:+.4f},{fp[2]:+.4f}) "
          f"fwd=({ff[0]:+.4f},{ff[1]:+.4f},{ff[2]:+.4f}) "
          f"up=({fu[0]:+.4f},{fu[1]:+.4f},{fu[2]:+.4f})", flush=True)
    print(f"  CMP rear  pos=({rp[0]:+.4f},{rp[1]:+.4f},{rp[2]:+.4f}) "
          f"fwd=({rf[0]:+.4f},{rf[1]:+.4f},{rf[2]:+.4f}) "
          f"up=({ru[0]:+.4f},{ru[1]:+.4f},{ru[2]:+.4f})", flush=True)
    print(f"  CMP expected_rear(Rz180*front) pos=({ep[0]:+.4f},{ep[1]:+.4f},{ep[2]:+.4f}) "
          f"fwd=({ef[0]:+.4f},{ef[1]:+.4f},{ef[2]:+.4f}) "
          f"up=({eu[0]:+.4f},{eu[1]:+.4f},{eu[2]:+.4f})", flush=True)
    return ok


def _author_rear_mirror(out, front_path, rear_path):
    """(폴백) cam_rear 가 없을 때 브리프 방식으로 저작: front 서브트리 복사 →
    프림 이름 front→rear 치환 → 링크 로컬변환 = Rz(180)·T_front.
    자식 서브트리를 그대로 복사하므로 카메라 월드 포즈는 Rz(180)·front 가 된다."""
    from pxr import Sdf, UsdGeom, Usd, Gf
    layer = out.GetRootLayer()
    Tf = UsdGeom.Xformable(out.GetPrimAtPath(front_path)).GetLocalTransformation(
        Usd.TimeCode.Default())
    if not Sdf.CopySpec(layer, front_path, layer, rear_path):
        raise RuntimeError(f"Sdf.CopySpec 실패 {front_path} -> {rear_path}")

    def rename(prim):
        for c in list(prim.GetChildren()):
            rename(c)
        nm = prim.GetName()
        if "front" in nm.lower():
            new = nm.replace("front", "rear").replace("Front", "Rear")
            e = Sdf.BatchNamespaceEdit()
            e.Add(prim.GetPath(), prim.GetPath().GetParentPath().AppendChild(new))
            if not layer.Apply(e):
                raise RuntimeError(f"이름 치환 실패 {prim.GetPath()} -> {new}")

    rename(out.GetPrimAtPath(rear_path))
    Rz = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), 180.0))
    xf = UsdGeom.Xformable(out.GetPrimAtPath(rear_path))
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(Tf * Rz)


def _joints_referencing(stage, link_path_str):
    """physics:body0/body1 이 주어진 링크를 참조하는 조인트 프림 경로 목록."""
    hits = []
    for p in stage.Traverse():
        if "Joint" not in p.GetTypeName():
            continue
        targets = []
        for rn in ("physics:body0", "physics:body1"):
            rel = p.GetRelationship(rn)
            if rel:
                targets += [str(t) for t in rel.GetTargets()]
        if any(link_path_str == t or t.startswith(link_path_str + "/") for t in targets):
            hits.append(str(p.GetPath()))
    return hits


def build(stage):
    """원본을 flatten 복사 → cam_rear 확보(기존 미러 유지/없으면 저작) →
    cam_qr_down_link(+관련 조인트) 제거 → OmniverseKit 카메라 제거 → 저장."""
    from pxr import Usd, Sdf

    flat = stage.Flatten()               # 자체포함 레이어(참조 없음)
    flat.Export(str(OUT))
    out = Usd.Stage.Open(str(OUT))

    front_link_path = ROBOT + "/cam_front_link"
    rear_link_path = ROBOT + "/cam_rear_link"
    qr_link_path = ROBOT + "/cam_qr_down_link"

    if not out.GetPrimAtPath(front_link_path).IsValid():
        raise RuntimeError("cam_front_link 없음")

    # 1) cam_rear 확보: 이미 존재하면 유지, 없으면 front 미러로 저작
    if out.GetPrimAtPath(rear_link_path).IsValid():
        print("REARCAM_NOTE cam_rear_link 이미 존재 → 유지(미러 검증은 하단 REARCAM_MIRROR)",
              flush=True)
    else:
        _author_rear_mirror(out, Sdf.Path(front_link_path), Sdf.Path(rear_link_path))
        print("REARCAM_NOTE cam_rear_link 신규 저작(front Rz(180) 미러)", flush=True)

    # 2) cam_qr_down 완전 제거. 이 로봇은 3중 계층(main 아티큘레이션 트리 /
    #    /visuals 프록시 / /colliders 프록시)을 가지므로 cam_qr_down_link 라는 이름의
    #    프림이 여러 곳에 있다. 관련 조인트 + 이름이 일치하는 모든 링크 프림을 제거한다.
    qr_joints = _joints_referencing(out, qr_link_path)
    for j in qr_joints:
        out.RemovePrim(Sdf.Path(j))
    qr_prims = [str(p.GetPath()) for p in out.Traverse()
                if p.GetName() == "cam_qr_down_link"]      # 제거 전 목록 확정
    for qp in qr_prims:
        if out.GetPrimAtPath(qp).IsValid():
            out.RemovePrim(Sdf.Path(qp))
    print(f"REARCAM_NOTE cam_qr_down 제거 joints={qr_joints} links={qr_prims}", flush=True)

    # 3) OmniverseKit 에디터 뷰포트 카메라 제거(로봇 카메라 아님 → 파일에 4개만 남김)
    removed_kit = [k for k in KIT_CAMS if out.GetPrimAtPath(k).IsValid()]
    for k in removed_kit:
        out.RemovePrim(Sdf.Path(k))
    print(f"REARCAM_NOTE OmniverseKit 에디터 카메라 제거 {removed_kit}", flush=True)

    out.GetRootLayer().Save()

    cams = [p for p in out.Traverse() if p.GetTypeName() == "Camera"]
    names = sorted(str(p.GetPath()) for p in cams)
    print(f"REARCAM_BUILT out={OUT.name} camera_count={len(cams)}", flush=True)
    for n in names:
        print(f"  OUT_CAMERA {n}", flush=True)

    fc = _robot_cam_path(out, "cam_front_link")
    rc = _robot_cam_path(out, "cam_rear_link")
    if fc and rc:
        _rear_mirror_check(out, fc, rc)


# ---------------------------------------------------------------------------
# 검증
# ---------------------------------------------------------------------------
def _disk_camera_specs():
    """합성/런타임 주입과 무관하게 저장된 레이어의 Camera 프림 스펙 경로."""
    from pxr import Sdf
    lyr = Sdf.Layer.FindOrOpen(str(OUT))
    found = []

    def walk(spec):
        for c in spec.nameChildren:
            if c.typeName == "Camera":
                found.append(c.path.pathString)
            walk(c)

    walk(lyr.pseudoRoot)
    return found


def verify():
    """새 파일을 재오픈해 로봇 카메라 4개(front/rear/left/right), cam_qr_down 부재를 검증.
    파일 실제 저장 내용(Sdf)과 합성 스테이지 양쪽으로 확인한다."""
    from pxr import Usd
    disk = _disk_camera_specs()                      # 디스크 실제 저장(권위)
    out = Usd.Stage.Open(str(OUT))
    allcams = [str(p.GetPath()) for p in out.Traverse() if p.GetTypeName() == "Camera"]
    robot = [p for p in allcams if p.startswith(ROBOT + "/")]
    kit = [p for p in allcams if p.startswith("/OmniverseKit")]
    has_rear = any("rear" in p.lower() for p in robot)
    has_front = any("front" in p.lower() for p in robot if "rear" not in p.lower())
    has_left = any("left" in p.lower() for p in robot)
    has_right = any("right" in p.lower() for p in robot)
    has_qr = any("qr_down" in p.lower() for p in allcams) or \
             any(x.GetName() == "cam_qr_down_link" for x in out.Traverse())
    ok = (len(disk) == 4 and len(robot) == 4 and has_rear and has_front
          and has_left and has_right and not has_qr)
    print(f"REARCAM_VERIFY={'PASS' if ok else 'FAIL'} n=4 n_disk={len(disk)} "
          f"n_robot={len(robot)} n_kit_runtime={len(kit)} rear={has_rear} "
          f"front={has_front} left={has_left} right={has_right} qr_down={has_qr} "
          f"paths={sorted(robot)}", flush=True)


def main():
    app = _boot()
    from pxr import Usd
    stage = Usd.Stage.Open(str(SRC))
    if "--inspect" in sys.argv[1:]:
        inspect(stage)
        app.close()
        return
    if "--deep" in sys.argv[1:]:
        deep_inspect(stage)
        app.close()
        return
    build(stage)
    verify()
    app.close()


if __name__ == "__main__":
    main()
