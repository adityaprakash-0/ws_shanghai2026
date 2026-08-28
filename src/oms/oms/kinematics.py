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
