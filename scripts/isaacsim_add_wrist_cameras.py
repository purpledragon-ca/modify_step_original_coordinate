"""
Isaac Sim Script Editor — Add wrist cameras to the aico2_x2 robot.

Paste this entire script into  Window > Script Editor  and click Run.

Camera prims are parented to Robot_left_Link7 / Robot_right_Link7 — the
links whose meshes (*_with_camera_and_wire.STL) already show the camera body.

Transform derivation
--------------------
  T_Link7→camera  =  T_Link7→flange  ×  T_flange→camera

  T_Link7→flange  comes from the URDF fixed joints:
    left  : xyz=(0,0, 0.081)  rpy=(0,   0, -π)
    right : xyz=(0,0,-0.081)  rpy=(π,   0, -π)

  T_flange→camera comes from the eye_in_hand calibration files:
    mars_core/src/mars/config/camera/calibrations/
      aico2_left_arm_D435_calibration.calib
      aico2_right_arm_D435_calibration.calib

Results
-------
  LEFT  xyz=( 0.07849, -0.03065,  0.17040) m
        rpy=(-0.11107, -0.00945,  1.57785) rad  ≈ (-6.4°, -0.5°, +90.4°)

  RIGHT xyz=(-0.07788, -0.03626, -0.19058) m
        rpy=( 3.04033,  0.00211,  1.58384) rad  ≈ (+174.2°, +0.1°, +90.7°)
        (roll≈180° = camera inverted on mirrored arm — correct for symmetric setup)
"""

import numpy as np
import omni.usd
from pxr import Gf, Usd, UsdGeom

# ── Stage ────────────────────────────────────────────────────────────────────
stage = omni.usd.get_context().get_stage()

# ── Robot prim base path ─────────────────────────────────────────────────────
# The articulation root when loaded via the URDF importer.
# Adjust if your stage uses a different path (check the Stage panel).
ROBOT_ROOT = "/World/aico2_x2"

# ── RealSense D435 lens parameters ───────────────────────────────────────────
D435_FOCAL_MM = 1.93    # focal length  (mm)
D435_HAPT_MM  = 3.683   # horiz aperture → HFoV ≈ 69.4°
D435_VAPT_MM  = 2.074   # vert  aperture → VFoV ≈ 42.5°
CLIP_NEAR     = 0.01    # near clip (m)
CLIP_FAR      = 10.0    # far  clip (m)

# ── Camera definitions ───────────────────────────────────────────────────────
# xyz / rpy are expressed in the Link7 frame (pre-computed above).
CAMERAS = {
    "left": {
        "link7_path":  f"{ROBOT_ROOT}/Robot_left_Link7",
        "camera_path": f"{ROBOT_ROOT}/Robot_left_Link7/left_camera",
        "frame_id":    "left_arm_color_optical_frame",
        "xyz": ( 0.07849, -0.03065,  0.17040),   # metres
        "rpy": (-3.03052,  0.00945, -1.56374),   # radians (ZYX) — +180° Y applied
    },
    "right": {
        "link7_path":  f"{ROBOT_ROOT}/Robot_right_Link7",
        "camera_path": f"{ROBOT_ROOT}/Robot_right_Link7/right_camera",
        "frame_id":    "right_arm_color_optical_frame",
        "xyz": (-0.07788, -0.03626, -0.19058),
        "rpy": ( 0.10126, -0.00211, -1.55775),   # radians (ZYX) — +180° Y applied
    },
}


# ── Helpers ──────────────────────────────────────────────────────────────────

def find_prim_fuzzy(stage, exact_path):
    """Try the exact path first, then search the stage by link name."""
    prim = stage.GetPrimAtPath(exact_path)
    if prim.IsValid():
        return prim, exact_path
    link_name = exact_path.split("/")[-1]
    for p in stage.Traverse():
        if p.GetName() == link_name:
            return p, str(p.GetPath())
    return None, None


def get_world_pos(stage, prim_path):
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        return None
    xfc = UsdGeom.XformCache()
    return xfc.GetLocalToWorldTransform(prim)


# ── Main ─────────────────────────────────────────────────────────────────────

print("=" * 62)
print("Adding wrist cameras  →  Robot_left/right_Link7")
print("=" * 62)

created = []

for side, cfg in CAMERAS.items():
    # ── Locate Link7 prim ────────────────────────────────────────────────
    link7_prim, resolved_path = find_prim_fuzzy(stage, cfg["link7_path"])
    if link7_prim is None:
        print(f"[WARN] {side}: Link7 prim not found at '{cfg['link7_path']}'")
        print( "       Camera created at configured path but won't follow arm.")
    else:
        # If found at a different path, update the camera path accordingly
        if resolved_path != cfg["link7_path"]:
            print(f"[INFO] {side}: found Link7 at  {resolved_path}")
            cfg["camera_path"] = resolved_path + f"/{side}_camera"

    # ── Create Camera prim ────────────────────────────────────────────────
    cam_prim = UsdGeom.Camera(stage.DefinePrim(cfg["camera_path"], "Camera"))
    if not cam_prim:
        print(f"[ERROR] {side}: could not create Camera at {cfg['camera_path']}")
        continue

    xform = UsdGeom.XformCommonAPI(cam_prim)

    # Local translation (metres) — relative to Link7 origin
    tx, ty, tz = cfg["xyz"]
    xform.SetTranslate(Gf.Vec3d(tx, ty, tz))

    # Local rotation — ZYX Euler in degrees
    roll_deg  = float(np.degrees(cfg["rpy"][0]))
    pitch_deg = float(np.degrees(cfg["rpy"][1]))
    yaw_deg   = float(np.degrees(cfg["rpy"][2]))
    xform.SetRotate(
        Gf.Vec3f(roll_deg, pitch_deg, yaw_deg),
        UsdGeom.XformCommonAPI.RotationOrderXYZ,
    )

    # ── D435 lens ─────────────────────────────────────────────────────────
    cam_prim.GetFocalLengthAttr().Set(D435_FOCAL_MM)
    cam_prim.GetHorizontalApertureAttr().Set(D435_HAPT_MM)
    cam_prim.GetVerticalApertureAttr().Set(D435_VAPT_MM)
    cam_prim.GetProjectionAttr().Set("perspective")
    cam_prim.GetClippingRangeAttr().Set(Gf.Vec2f(CLIP_NEAR, CLIP_FAR))

    # ── Metadata ──────────────────────────────────────────────────────────
    p = cam_prim.GetPrim()
    p.SetCustomDataByKey("ros_frame_id",     cfg["frame_id"])
    p.SetCustomDataByKey("parent_link",      cfg["link7_path"].split("/")[-1])
    p.SetCustomDataByKey("calibration_type", "eye_in_hand  (flange→optical, composed to Link7)")

    created.append(cfg["camera_path"])

    # ── World pose for cross-check ────────────────────────────────────────
    world_mat = get_world_pos(stage, resolved_path or cfg["link7_path"])
    if world_mat:
        cam_world = world_mat.Transform(Gf.Vec3d(tx, ty, tz))
        print(f"\n[OK]  {side.upper()} camera")
        print(f"      Stage path   : {cfg['camera_path']}")
        print(f"      ROS frame    : {cfg['frame_id']}")
        print(f"      Local xyz    : ({tx:.5f}, {ty:.5f}, {tz:.5f}) m  [Link7 frame]")
        print(f"      Local RPY°   : ({roll_deg:.2f}, {pitch_deg:.2f}, {yaw_deg:.2f})")
        print(f"      World xyz    : ({cam_world[0]:.4f}, {cam_world[1]:.4f}, {cam_world[2]:.4f}) m")
    else:
        print(f"\n[OK]  {side.upper()} camera at {cfg['camera_path']}")

print("\n" + "=" * 62)
print(f"Done. {len(created)} camera(s) created.")
print("=" * 62)
print("""
To verify in Isaac Sim:
  • Stage panel: expand Robot_left_Link7 → left_camera
                          Robot_right_Link7 → right_camera
  • Viewport top-left  → Camera → left_camera  (look through frustum)
  • Move joint7 slider → cameras should follow the wrist mesh exactly.
  • If the view is pointing backward, the Isaac Sim camera convention
    (looks down -Z) is opposite to ROS optical (+Z); rotate 180° around X.
""")
