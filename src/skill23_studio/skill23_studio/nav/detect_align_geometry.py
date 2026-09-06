"""Orbbec depth geometry for Detect+Align.

Orbbec driver (/camera/depth/image_raw):
  - Format: 16UC1, values in millimeters (raw Y16 * depth_scale in driver).
  - Meaning: Z-depth along the **depth camera optical Z axis** (not slant range).
  - Frame: camera_depth_optical_frame (REP-103: X right, Y down, Z forward).

Color (/camera/color/image_raw): RGB8/MJPG decode, camera_color_optical_frame.
YOLO runs on color; depth is sampled at the matching pixel (same 640x360 layout).

Camera mount: pitched down ``tilt_deg`` from horizontal (robot +X forward, +Z up).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

from sensor_msgs.msg import CameraInfo


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int


@dataclass(frozen=True)
class TargetGeometry:
    """3D target metrics in robot horizontal plane (origin = camera optical center)."""

    z_depth_m: float
    xc: float
    yc: float
    zc: float
    forward_m: float
    lateral_m: float
    floor_dist_m: float
    bearing_deg: float  # rotation to align image center with object (OMS yaw)
    yaw_deg: float


def pitch_rad_from_param(tilt_deg: float, from_vertical: bool = False) -> float:
    """Convert config tilt to pitch-down-from-horizontal radians.

    ``from_vertical=True``: tilt_deg is measured down from robot +Z (vertical).
    ``from_vertical=False`` (default): tilt_deg is measured down from horizontal.
    """
    t = float(tilt_deg)
    if from_vertical:
        t = 90.0 - t
    return math.radians(max(0.0, min(89.9, t)))


def intrinsics_from_camera_info(msg: CameraInfo) -> CameraIntrinsics:
    k = msg.k
    return CameraIntrinsics(
        fx=float(k[0]),
        fy=float(k[4]),
        cx=float(k[2]),
        cy=float(k[5]),
        width=int(msg.width),
        height=int(msg.height),
    )


def intrinsics_fallback(width: int, height: int, hfov_deg: float) -> CameraIntrinsics:
    half = math.radians(max(10.0, float(hfov_deg)) * 0.5)
    fx = (float(width) * 0.5) / math.tan(half)
    fy = fx
    return CameraIntrinsics(
        fx=fx,
        fy=fy,
        cx=float(width) * 0.5,
        cy=float(height) * 0.5,
        width=int(width),
        height=int(height),
    )


def bbox_center_px(obj) -> Tuple[float, float]:
    u = 0.5 * (float(obj.x1) + float(obj.x2))
    v = 0.5 * (float(obj.y1) + float(obj.y2))
    return u, v


def u_norm_from_pixel(u_px: float, width: int, u_off: float = 0.0) -> float:
    w = float(width)
    if w <= 1.0:
        return 0.0
    return (u_px - 0.5 * w) / (0.5 * w) - float(u_off)


def map_color_pixel_to_depth(
        u_color: float, v_color: float,
        color: CameraIntrinsics, depth: CameraIntrinsics,
) -> Tuple[float, float]:
    if color.width == depth.width and color.height == depth.height:
        return u_color, v_color
    sx = float(depth.width) / max(1.0, float(color.width))
    sy = float(depth.height) / max(1.0, float(color.height))
    return u_color * sx, v_color * sy


def unproject_z_depth(
        u_px: float, v_px: float, z_m: float, intr: CameraIntrinsics,
) -> Tuple[float, float, float]:
    """Pinhole unproject using Orbbec Z-depth (meters)."""
    zc = float(z_m)
    xc = (float(u_px) - intr.cx) * zc / intr.fx
    yc = (float(v_px) - intr.cy) * zc / intr.fy
    return xc, yc, zc


def camera_point_to_robot_horizontal(
        xc: float, yc: float, zc: float, pitch_rad: float,
) -> Tuple[float, float, float]:
    """Camera optical point → robot horizontal plane (+X fwd, +Y left, +Z up).

    Camera pitched down ``pitch_rad`` from horizontal about robot +Y.
    Derived from R = R_y(pitch) @ R_level,  R_level: cam→robot at zero pitch.

    Returns (forward_m, lateral_m, height_up_m).
    """
    c = math.cos(pitch_rad)
    s = math.sin(pitch_rad)
    forward = zc * c - yc * s
    lateral = -xc
    height_up = -(zc * s + yc * c)
    return forward, lateral, height_up


def bearing_deg_from_pixel(
        u_px: float, intr: CameraIntrinsics,
        asign: float = 1.0,
        oms_min: float = -90.0, oms_max: float = 90.0,
) -> float:
    """Relative yaw correction from the **current** camera view (+ACW / −CW).

    This is how much more to rotate OMS so the object moves to image center.
    It is NOT an absolute goto angle — add to current OMS position before commanding.
    """
    raw = math.degrees(math.atan2(float(u_px) - intr.cx, intr.fx))
    yaw = -float(asign) * raw
    return max(float(oms_min), min(float(oms_max), yaw))


def oms_goto_target_deg(
        oms_now_deg: float, bearing_deg: float,
        oms_min: float = -90.0, oms_max: float = 90.0,
) -> float:
    """Absolute OMS goto = current position + relative bearing correction."""
    return max(float(oms_min), min(float(oms_max), float(oms_now_deg) + float(bearing_deg)))


def yaw_deg_from_horizontal(
        forward_m: float, lateral_m: float,
        oms_min: float = -90.0, oms_max: float = 90.0,
) -> float:
    """Azimuth on the robot floor plane: atan2(lateral, forward)."""
    if forward_m <= 0.01:
        if abs(lateral_m) < 1e-4:
            yaw = 0.0
        else:
            yaw = 90.0 if lateral_m > 0.0 else -90.0
    else:
        yaw = math.degrees(math.atan2(lateral_m, forward_m))
    return max(float(oms_min), min(float(oms_max), yaw))


def yaw_direction(yaw_deg: float) -> str:
    if yaw_deg > 0.05:
        return 'ACW'
    if yaw_deg < -0.05:
        return 'CW'
    return 'centered'


def compute_target_geometry(
        u_color_px: float, v_color_px: float, z_depth_m: float,
        color_intr: CameraIntrinsics, depth_intr: CameraIntrinsics,
        pitch_rad: float,
        asign: float = 1.0,
        oms_min: float = -90.0, oms_max: float = 90.0,
) -> TargetGeometry:
    """Full geometry from color bbox + depth Z + intrinsics + mount pitch."""
    bearing = bearing_deg_from_pixel(
        u_color_px, color_intr, asign, oms_min, oms_max)
    u_d, v_d = map_color_pixel_to_depth(
        u_color_px, v_color_px, color_intr, depth_intr)
    xc, yc, zc = unproject_z_depth(u_d, v_d, z_depth_m, depth_intr)
    forward, lateral, _height = camera_point_to_robot_horizontal(
        xc, yc, zc, pitch_rad)
    floor_dist = math.hypot(forward, lateral)
    yaw_floor = yaw_deg_from_horizontal(forward, lateral, oms_min, oms_max)
    return TargetGeometry(
        z_depth_m=float(z_depth_m),
        xc=xc, yc=yc, zc=zc,
        forward_m=forward,
        lateral_m=lateral,
        floor_dist_m=floor_dist,
        bearing_deg=bearing,
        yaw_deg=yaw_floor,
    )


# Backward-compatible alias
FloorPoint = TargetGeometry
compute_floor_point = compute_target_geometry


class CameraInfoCache:
    """Latest /camera/*/camera_info with HFOV fallback."""

    def __init__(self, hfov_deg: float = 69.0) -> None:
        self._hfov = float(hfov_deg)
        self._color: Optional[CameraIntrinsics] = None
        self._depth: Optional[CameraIntrinsics] = None

    def set_color_info(self, msg: CameraInfo) -> None:
        if msg.width > 0 and msg.height > 0 and msg.k[0] > 1e-3:
            self._color = intrinsics_from_camera_info(msg)

    def set_depth_info(self, msg: CameraInfo) -> None:
        if msg.width > 0 and msg.height > 0 and msg.k[0] > 1e-3:
            self._depth = intrinsics_from_camera_info(msg)

    def color_or_fallback(self, width: int, height: int) -> CameraIntrinsics:
        if self._color is not None:
            return self._color
        return intrinsics_fallback(width, height, self._hfov)

    def depth_or_fallback(self, width: int, height: int) -> CameraIntrinsics:
        if self._depth is not None:
            return self._depth
        return intrinsics_fallback(width, height, self._hfov)
