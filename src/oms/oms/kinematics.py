"""OMS rotation kinematics.

Motor: Studica Maverick Hex Planetary Gearbox with encoder, 71.2:1
https://www.studica.com/maverick-hex-planetary-gearbox-motor-wencoder-7121-ratio-gearbox

Epicyclic layout: 120 T sun (fixed to chassis) + 32 T pinion on gearbox
output, motor bolted to the carrier (OMS turret).  The built-in encoder
measures pinion rotation *relative to the carrier*, not the world frame.

With stationary sun (Willis, ω_sun = 0):

    ω_carrier = ω_motor_output × (N_pinion / N_sun)

Turret ratio R_t = N_sun / N_pinion = 120 / 32 = 3.75.

The external-mesh "+1" term (N_sun + N_pinion) / N_pinion does NOT apply
here — that counts total pinion spins from a world-fixed observer.
"""

from __future__ import annotations

import math

ENCODER_CPR = 6.0
QUADRATURE = 4.0
GEARBOX_RATIO = 71.2
PINION_TEETH = 32
SUN_TEETH = 120
SUN_DIAMETER_MM = 122.0


def turret_ratio(
        sun_teeth: float = SUN_TEETH,
        pinion_teeth: float = PINION_TEETH) -> float:
    """Carrier rotation per motor-output revolution (encoder on carrier)."""
    pinion = max(1.0, float(pinion_teeth))
    return float(sun_teeth) / pinion


def ticks_per_motor_rev(
        encoder_cpr: float = ENCODER_CPR,
        quadrature: float = QUADRATURE,
        gearbox_ratio: float = GEARBOX_RATIO) -> float:
    return float(encoder_cpr) * float(quadrature) * float(gearbox_ratio)


def ticks_per_turret_rev(
        encoder_cpr: float = ENCODER_CPR,
        quadrature: float = QUADRATURE,
        gearbox_ratio: float = GEARBOX_RATIO,
        sun_teeth: float = SUN_TEETH,
        pinion_teeth: float = PINION_TEETH) -> float:
    return ticks_per_motor_rev(
        encoder_cpr, quadrature, gearbox_ratio) * turret_ratio(
            sun_teeth, pinion_teeth)


def ticks_per_oms_deg(
        encoder_cpr: float = ENCODER_CPR,
        quadrature: float = QUADRATURE,
        gearbox_ratio: float = GEARBOX_RATIO,
        sun_teeth: float = SUN_TEETH,
        pinion_teeth: float = PINION_TEETH) -> float:
    return ticks_per_turret_rev(
        encoder_cpr, quadrature, gearbox_ratio, sun_teeth, pinion_teeth) / 360.0


def ticks_to_deg(ticks: float, ticks_per_deg: float) -> float:
    scale = float(ticks_per_deg)
    if abs(scale) < 1e-9:
        return 0.0
    return float(ticks) / scale


def deg_to_ticks(deg: float, ticks_per_deg: float) -> float:
    return float(deg) * float(ticks_per_deg)


# --- Lift (Titan1 M1): OMS Lift Odometry (WorldSkills PDF) ---
#
# Drive: 99.5:1 Maverick → 13/26 bevel → 32/64 spur → #25 chain, 24T sprocket.
#   height_mm = ticks × (p × N × k) / (Cm × Q × G × Rb × Rs)
#   ticks     = height_mm × TICKS_PER_MM
#
# Reference (k=1 direct clamp, 24T sprocket):
#   T_rev (sprocket) = 9552 ticks/rev
#   MM_PER_REV       = 152.40 mm/rev  (= 6.35 × 24)
#   MM_PER_TICK      = 0.015955 mm
#   TICKS_PER_MM     = 62.677

LIFT_GEARBOX_RATIO = 99.5
LIFT_BEVEL_PINION = 13.0
LIFT_BEVEL_GEAR = 26.0
LIFT_SPUR_PINION = 32.0
LIFT_SPUR_GEAR = 64.0
LIFT_SPROCKET_TEETH = 24.0
LIFT_CHAIN_PITCH_MM = 6.35  # #25 = 0.25 inch
LIFT_CHAIN_STAGES = 1.0     # 2.0 if cascading carriage (PDF item #1)


def lift_bevel_ratio(
        bevel_pinion: float = LIFT_BEVEL_PINION,
        bevel_gear: float = LIFT_BEVEL_GEAR) -> float:
    """Rb = driven gear / drive pinion (26/13 = 2.0)."""
    return max(1.0, float(bevel_gear)) / max(1.0, float(bevel_pinion))


def lift_spur_ratio(
        spur_pinion: float = LIFT_SPUR_PINION,
        spur_gear: float = LIFT_SPUR_GEAR) -> float:
    """Rs = driven gear / drive pinion (64/32 = 2.0)."""
    return max(1.0, float(spur_gear)) / max(1.0, float(spur_pinion))


def lift_ticks_per_sprocket_rev(
        encoder_cpr: float = ENCODER_CPR,
        quadrature: float = QUADRATURE,
        gearbox_ratio: float = LIFT_GEARBOX_RATIO,
        bevel_pinion: float = LIFT_BEVEL_PINION,
        bevel_gear: float = LIFT_BEVEL_GEAR,
        spur_pinion: float = LIFT_SPUR_PINION,
        spur_gear: float = LIFT_SPUR_GEAR) -> float:
    """T_rev = Cm × Q × G × Rb × Rs  (encoder counts per sprocket revolution)."""
    return (
        float(encoder_cpr) * float(quadrature) * float(gearbox_ratio)
        * lift_bevel_ratio(bevel_pinion, bevel_gear)
        * lift_spur_ratio(spur_pinion, spur_gear))


def lift_chain_mm_per_sprocket_rev(
        sprocket_teeth: float = LIFT_SPROCKET_TEETH,
        chain_pitch_mm: float = LIFT_CHAIN_PITCH_MM,
        chain_stages: float = LIFT_CHAIN_STAGES) -> float:
    """MM_PER_REV = p × N × k  (linear carriage travel per sprocket revolution)."""
    teeth = max(1.0, float(sprocket_teeth))
    pitch = max(0.1, float(chain_pitch_mm))
    stages = max(1.0, float(chain_stages))
    return teeth * pitch * stages


def lift_mm_per_tick(
        encoder_cpr: float = ENCODER_CPR,
        quadrature: float = QUADRATURE,
        gearbox_ratio: float = LIFT_GEARBOX_RATIO,
        sprocket_teeth: float = LIFT_SPROCKET_TEETH,
        chain_pitch_mm: float = LIFT_CHAIN_PITCH_MM,
        chain_stages: float = LIFT_CHAIN_STAGES,
        bevel_pinion: float = LIFT_BEVEL_PINION,
        bevel_gear: float = LIFT_BEVEL_GEAR,
        spur_pinion: float = LIFT_SPUR_PINION,
        spur_gear: float = LIFT_SPUR_GEAR) -> float:
    """MM_PER_TICK = (p × N × k) / T_rev."""
    t_rev = lift_ticks_per_sprocket_rev(
        encoder_cpr, quadrature, gearbox_ratio,
        bevel_pinion, bevel_gear, spur_pinion, spur_gear)
    if t_rev < 1e-9:
        return 0.0
    return lift_chain_mm_per_sprocket_rev(
        sprocket_teeth, chain_pitch_mm, chain_stages) / t_rev


def ticks_per_lift_mm(
        encoder_cpr: float = ENCODER_CPR,
        quadrature: float = QUADRATURE,
        gearbox_ratio: float = LIFT_GEARBOX_RATIO,
        sprocket_teeth: float = LIFT_SPROCKET_TEETH,
        chain_pitch_mm: float = LIFT_CHAIN_PITCH_MM,
        chain_stages: float = LIFT_CHAIN_STAGES,
        bevel_pinion: float = LIFT_BEVEL_PINION,
        bevel_gear: float = LIFT_BEVEL_GEAR,
        spur_pinion: float = LIFT_SPUR_PINION,
        spur_gear: float = LIFT_SPUR_GEAR) -> float:
    """TICKS_PER_MM = T_rev / (p × N × k).  PDF default ≈ 47.008."""
    mm_rev = lift_chain_mm_per_sprocket_rev(
        sprocket_teeth, chain_pitch_mm, chain_stages)
    if mm_rev < 1e-9:
        return 0.0
    return lift_ticks_per_sprocket_rev(
        encoder_cpr, quadrature, gearbox_ratio,
        bevel_pinion, bevel_gear, spur_pinion, spur_gear) / mm_rev


def lift_ticks_to_mm(
        ticks: float,
        ticks_per_mm: float = 0.0,
        **kinematics) -> float:
    if ticks_per_mm > 0.0:
        return float(ticks) / float(ticks_per_mm)
    return float(ticks) * lift_mm_per_tick(**kinematics)


def lift_mm_to_ticks(
        mm: float,
        ticks_per_mm: float = 0.0,
        **kinematics) -> int:
    if ticks_per_mm > 0.0:
        return int(round(float(mm) * float(ticks_per_mm)))
    return int(round(float(mm) * ticks_per_lift_mm(**kinematics)))


# Backward-compatible aliases (belt naming — do not use for chain lift).
def lift_drive_rev_per_motor_rev(
        bevel_pinion: float = LIFT_BEVEL_PINION,
        bevel_gear: float = LIFT_BEVEL_GEAR,
        spur_pinion: float = LIFT_SPUR_PINION,
        spur_gear: float = LIFT_SPUR_GEAR) -> float:
    return 1.0 / (
        lift_bevel_ratio(bevel_pinion, bevel_gear)
        * lift_spur_ratio(spur_pinion, spur_gear))


def lift_pulley_rev_per_motor_rev(
        bevel_pinion: float = LIFT_BEVEL_PINION,
        bevel_gear: float = LIFT_BEVEL_GEAR,
        spur_pinion: float = LIFT_SPUR_PINION,
        spur_gear: float = LIFT_SPUR_GEAR) -> float:
    return lift_drive_rev_per_motor_rev(
        bevel_pinion, bevel_gear, spur_pinion, spur_gear)


def lift_mm_per_motor_rev(
        sprocket_teeth: float = LIFT_SPROCKET_TEETH,
        chain_pitch_mm: float = LIFT_CHAIN_PITCH_MM,
        bevel_pinion: float = LIFT_BEVEL_PINION,
        bevel_gear: float = LIFT_BEVEL_GEAR,
        spur_pinion: float = LIFT_SPUR_PINION,
        spur_gear: float = LIFT_SPUR_GEAR,
        *,
        chain_stages: float = LIFT_CHAIN_STAGES,
        sprocket_pitch_diameter_mm: float = 0.0,
        pulley_diameter_mm: float = 0.0,
        pulley_teeth: float = 0.0,
        belt_pitch_mm: float = 0.0) -> float:
    drive_rev = lift_drive_rev_per_motor_rev(
        bevel_pinion, bevel_gear, spur_pinion, spur_gear)
    return drive_rev * lift_chain_mm_per_sprocket_rev(
        sprocket_teeth or pulley_teeth or LIFT_SPROCKET_TEETH,
        chain_pitch_mm or belt_pitch_mm or LIFT_CHAIN_PITCH_MM,
        chain_stages)


def ticks_to_mm(ticks: float, ticks_per_mm: float) -> float:
    scale = float(ticks_per_mm)
    if abs(scale) < 1e-9:
        return 0.0
    return float(ticks) / scale


def mm_to_ticks(mm: float, ticks_per_mm: float) -> float:
    return float(mm) * float(ticks_per_mm)


# --- Linear (Cypher Max on Titan1 M3, quadrature) ---
# Studica Cypher Max: 512 CPR × 4 (quadrature) = 2048 counts/rev.
# Titan reset_encoder on M3 clears the quadrature counter (same as M2 rotation).

CYPHER_MAX_CPR = 512.0
CYPHER_QUADRATURE = 4.0


def cypher_ticks_per_rev(
        encoder_cpr: float = CYPHER_MAX_CPR,
        quadrature: float = CYPHER_QUADRATURE) -> float:
    return float(encoder_cpr) * float(quadrature)


def ticks_per_linear_mm(
        encoder_cpr: float = CYPHER_MAX_CPR,
        quadrature: float = CYPHER_QUADRATURE,
        mm_per_rev: float = 50.0) -> float:
    """mm_per_rev = linear travel for one Cypher revolution (calibrate on bench)."""
    mm = max(0.1, float(mm_per_rev))
    return cypher_ticks_per_rev(encoder_cpr, quadrature) / mm


# --- Studio 3D / analysis frame (metres, right-handed) ---
# Origin = OMS rotation centre on the turntable deck (Z = 0 is ground).
# +X = gripper forward (front), +Z = up, +Y = left (ACW from +X).
# Lift home = 0 mm at the UP limit. Positive lift_mm travels down.
# Linear home = 0 mm at rear limit (World X = linear_home_x_mm = 220).
# Max linear stroke = 110 mm forward → reach 330 mm from rotation centre.
# World Z of the linear carriage = z_top − lift_mm (600 at top, 360 at floor).

VIZ_LIFT_UPPER_Z_MM = 455.0
VIZ_LIFT_LOWER_Z_MM = 155.0
VIZ_LIFT_TRAVEL_MM = 300.0
VIZ_Z_TOP_MM = VIZ_LIFT_UPPER_Z_MM
VIZ_Z_FLOOR_MM = VIZ_LIFT_LOWER_Z_MM
VIZ_LINEAR_TRAVEL_MM = 110.0
VIZ_LINEAR_HOME_X_MM = 220.0
VIZ_LINEAR_MAX_REACH_MM = 330.0
VIZ_LINEAR_MAX_EXTENSION_MM = 110.0
VIZ_LINEAR_REF_Z_LOWER_MM = 110.0
VIZ_LINEAR_REF_Z_UPPER_MM = 375.0
VIZ_TOTAL_HEIGHT_MM = 575.0
VIZ_RAIL_HEIGHT_MM = 575.0
VIZ_TURNTABLE_DIAMETER_MM = 180.0
VIZ_CAMERA_TILT_DEG = 35.0
VIZ_GRIPPER_SERVO_MM = 110.0
VIZ_GRIPPER_LENGTH_MM = 200.0


def gripper_xyz_m(
        rotation_deg: float = 0.0,
        lift_mm: float = 0.0,
        linear_mm: float = 0.0,
        *,
        lift_travel_mm: float = VIZ_LIFT_TRAVEL_MM,
        lift_upper_z_mm: float = VIZ_LIFT_UPPER_Z_MM,
        lift_lower_z_mm: float = VIZ_LIFT_LOWER_Z_MM,
        linear_home_x_mm: float = VIZ_LINEAR_HOME_X_MM,
        linear_travel_mm: float = VIZ_LINEAR_TRAVEL_MM,
        linear_max_reach_mm: float = VIZ_LINEAR_MAX_REACH_MM,
        linear_max_extension_mm: float = VIZ_LINEAR_MAX_EXTENSION_MM,
        z_top_mm: float = VIZ_Z_TOP_MM,
        z_floor_mm: float = VIZ_Z_FLOOR_MM) -> tuple[float, float, float]:
    """Linear-carriage reference in OMS world frame (metres).

    lift_mm = down from upper home. Z = lift_upper − lift_offset.
    """
    upper = float(lift_upper_z_mm or z_top_mm)
    lower = float(lift_lower_z_mm or z_floor_mm)
    travel = max(1.0, float(lift_travel_mm) or (upper - lower))
    lift = max(0.0, min(travel, float(lift_mm)))
    z = (upper - lift) / 1000.0
    stroke_max = max(1.0, float(linear_travel_mm))
    if linear_max_extension_mm > 0.0:
        stroke_max = min(stroke_max, float(linear_max_extension_mm))
    reach_cap = float(linear_max_reach_mm) - float(linear_home_x_mm)
    if reach_cap > 1.0:
        stroke_max = min(stroke_max, reach_cap)
    stroke = max(0.0, min(stroke_max, float(linear_mm)))
    r = (float(linear_home_x_mm) + stroke) / 1000.0
    yaw = math.radians(float(rotation_deg))
    return r * math.cos(yaw), r * math.sin(yaw), z


def oms_analysis(
        rotation_deg: float = 0.0,
        lift_mm: float = 0.0,
        linear_mm: float = 0.0,
        **dims) -> dict:
    keys = (
        'lift_travel_mm', 'lift_upper_z_mm', 'lift_lower_z_mm',
        'linear_home_x_mm', 'linear_travel_mm', 'linear_max_extension_mm',
        'linear_max_reach_mm', 'z_top_mm', 'z_floor_mm',
    )
    x, y, z = gripper_xyz_m(rotation_deg, lift_mm, linear_mm, **{
        k: dims[k] for k in keys if k in dims
    })
    z_top = float(dims.get('z_top_mm', VIZ_Z_TOP_MM))
    z_floor = float(dims.get('z_floor_mm', VIZ_Z_FLOOR_MM))
    return {
        'x_m': round(x, 4),
        'y_m': round(y, 4),
        'z_m': round(z, 4),
        'x_mm': round(x * 1000.0, 1),
        'y_mm': round(y * 1000.0, 1),
        'z_mm': round(z * 1000.0, 1),
        'yaw_deg': round(float(rotation_deg), 2),
        'lift_mm': round(float(lift_mm), 1),
        'linear_mm': round(float(linear_mm), 1),
        'z_top_mm': z_top,
        'z_floor_mm': z_floor,
        'front': '+X',
    }
