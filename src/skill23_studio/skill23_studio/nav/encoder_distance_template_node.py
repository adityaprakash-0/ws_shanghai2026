#!/usr/bin/env python3
"""TEMPLATE — distance from one motor encoder (ticks → metres).

Copy / adapt this node when a motor, gear ratio, or encoder CPR changes.
It does **not** replace robot odometry; it teaches the calculation and
publishes a simple moved-distance for one wheel/axis.

------------------------------------------------------------------------
STEP-BY-STEP CALCULATION (edit the ROS params, not the formulas)
------------------------------------------------------------------------

1) Encoder counts per motor shaft revolution
     ticks_per_motor_rev
       = encoder_cpr          # counts per encoder rev (datasheet)
         * encoder_quad       # 1 if already quadrature-decoded, else 4
       Example: 12 CPR magnetic, x4 decode → 48 ticks / motor rev

2) Gear / sprocket reduction (motor revs per OUTPUT shaft rev)
     gear_ratio = N_out / N_motor   OR   teeth_driven / teeth_driver
       Example: 50-tooth wheel sprocket / 10-tooth motor sprocket → 5.0
       Meaning: 5 motor revolutions → 1 wheel revolution

3) Output (wheel) circumference
     wheel_circumference_m = 2 * pi * wheel_radius_m
       Example: radius 0.05 m → 0.314159 m per wheel rev

4) Metres per encoder tick (the key constant)
     meters_per_tick =
         wheel_circumference_m
         / (ticks_per_motor_rev * gear_ratio)

     Derivation:
       1 tick  →  1 / ticks_per_motor_rev  motor revs
               →  that / gear_ratio        wheel revs
               →  * circumference          metres

5) Distance moved since reset
     distance_m = (ticks_now - ticks_at_reset) * meters_per_tick * sign

WORKED EXAMPLE (matches defaults below)
  encoder_cpr          = 12
  encoder_quad         = 4          → ticks_per_motor_rev = 48
  gear_ratio           = 5.0
  wheel_radius_m       = 0.05       → circ ≈ 0.31416 m
  meters_per_tick      = 0.31416 / (48 * 5) ≈ 0.001309 m/tick
  If encoder advances +1000 ticks → distance ≈ 1.309 m

------------------------------------------------------------------------
Run (after sourcing the workspace)::

  ros2 run skill23_studio studio_encoder_distance_template --ros-args \\
    -p encoder_topic:=/titan0/m_0/encoder \\
    -p encoder_cpr:=12 -p encoder_quad:=4 \\
    -p gear_ratio:=5.0 -p wheel_radius_m:=0.05

Reset cumulative distance::

  ros2 topic pub --once /encoder_distance/reset std_msgs/msg/Empty '{}'
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Float64


class EncoderDistanceTemplate(Node):
    def __init__(self) -> None:
        super().__init__('encoder_distance_template')

        # --- Configure these when the hardware changes -----------------
        self.declare_parameter('encoder_topic', '/titan0/m_0/encoder')
        self.declare_parameter('encoder_cpr', 12.0)       # datasheet CPR
        self.declare_parameter('encoder_quad', 4.0)       # 1 or 4
        self.declare_parameter('gear_ratio', 5.0)         # motor revs / wheel rev
        self.declare_parameter('wheel_radius_m', 0.05)    # metres
        self.declare_parameter('sign', 1.0)               # +1 / -1 if inverted
        self.declare_parameter('distance_topic', '/encoder_distance/distance_m')
        self.declare_parameter('publish_hz', 20.0)
        # ----------------------------------------------------------------

        topic = str(self.get_parameter('encoder_topic').value)
        cpr = max(1e-9, float(self.get_parameter('encoder_cpr').value))
        quad = max(1.0, float(self.get_parameter('encoder_quad').value))
        gear = max(1e-9, float(self.get_parameter('gear_ratio').value))
        radius = max(1e-9, float(self.get_parameter('wheel_radius_m').value))
        self._sign = 1.0 if float(self.get_parameter('sign').value) >= 0.0 else -1.0

        ticks_per_motor_rev = cpr * quad
        circumference = 2.0 * math.pi * radius
        self._meters_per_tick = circumference / (ticks_per_motor_rev * gear)

        self._ticks: float | None = None
        self._ticks0: float | None = None
        self._distance_m = 0.0

        dist_topic = str(self.get_parameter('distance_topic').value)
        self._pub = self.create_publisher(Float64, dist_topic, 10)
        self._mpt_pub = self.create_publisher(
            Float64, '/encoder_distance/meters_per_tick', 10)
        self.create_subscription(Float64, topic, self._on_encoder, 10)
        self.create_subscription(Empty, '/encoder_distance/reset', self._on_reset, 10)

        hz = max(1.0, float(self.get_parameter('publish_hz').value))
        self.create_timer(1.0 / hz, self._tick)

        self.get_logger().info(
            f'TEMPLATE encoder={topic}  '
            f'ticks/motor_rev={ticks_per_motor_rev:.1f}  '
            f'gear={gear:.3f}  circ={circumference:.5f} m  '
            f'meters_per_tick={self._meters_per_tick:.9f}  '
            f'→ {dist_topic}')
        self.get_logger().info(
            'Formula: meters_per_tick = (2*pi*radius) / (cpr*quad*gear_ratio)')

    def _on_encoder(self, msg: Float64) -> None:
        self._ticks = float(msg.data)
        if self._ticks0 is None:
            self._ticks0 = self._ticks

    def _on_reset(self, _msg: Empty) -> None:
        if self._ticks is not None:
            self._ticks0 = self._ticks
        self._distance_m = 0.0
        self.get_logger().info('encoder distance reset')

    def _tick(self) -> None:
        self._mpt_pub.publish(Float64(data=float(self._meters_per_tick)))
        if self._ticks is None or self._ticks0 is None:
            return
        delta_ticks = self._ticks - self._ticks0
        self._distance_m = self._sign * delta_ticks * self._meters_per_tick
        self._pub.publish(Float64(data=float(self._distance_m)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EncoderDistanceTemplate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
