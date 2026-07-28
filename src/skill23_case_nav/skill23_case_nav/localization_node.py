"""Localization node — port of src/localization_node.cpp."""

from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster

from skill23_case_nav.utils import normalize_angle, quat_from_yaw, yaw_from_quat


class LocalizationNode(Node):
    def __init__(self) -> None:
        super().__init__("cn_localization")

        self.declare_parameter("map_frame", "map")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("odom_topic", "/odometry/filtered")
        self.declare_parameter("wheel_odom_topic", "/wheel/odom")
        self.declare_parameter("imu_topic", "/imu")
        self.declare_parameter("slip_wheel_vx_min", 0.08)
        self.declare_parameter("slip_accel_max", 0.25)
        self.declare_parameter("slip_hold_count", 5)

        self.map_frame_ = str(self.get_parameter("map_frame").value)
        self.odom_frame_ = str(self.get_parameter("odom_frame").value)
        self.base_frame_ = str(self.get_parameter("base_frame").value)
        self.slip_vx_ = float(self.get_parameter("slip_wheel_vx_min").value)
        self.slip_accel_ = float(self.get_parameter("slip_accel_max").value)
        self.slip_hold_ = int(self.get_parameter("slip_hold_count").value)

        self.seeded_ = False
        self.map_x_ = 0.0
        self.map_y_ = 0.0
        self.map_yaw_ = 0.0
        self.odom_x0_ = 0.0
        self.odom_y0_ = 0.0
        self.odom_yaw0_ = 0.0
        self.last_good_mx_ = 0.0
        self.last_good_my_ = 0.0
        self.imu_yaw0_ = 0.0
        self.ax_ = 0.0
        self.ay_ = 0.0
        self.slip_count_ = 0
        self.slipping_ = False

        self.imu_yaw_: Optional[float] = None
        self.filt_: Optional[Odometry] = None
        self.wheel_: Optional[Odometry] = None

        self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self._on_initialpose, 10
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._on_filtered,
            20,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("wheel_odom_topic").value),
            self._on_wheel,
            20,
        )
        self.create_subscription(
            Imu,
            str(self.get_parameter("imu_topic").value),
            self._on_imu,
            50,
        )

        self.pose_pub_ = self.create_publisher(PoseStamped, "/case_nav/pose", 10)
        self.odom_pub_ = self.create_publisher(Odometry, "/case_nav/odom", 10)
        self.tf_broadcaster_ = TransformBroadcaster(self)

        self.create_timer(0.05, self._tick)
        self.get_logger().info("Case-nav localization: EKF + IMU slip freeze")

    def _on_initialpose(self, msg: PoseWithCovarianceStamped) -> None:
        self.map_x_ = float(msg.pose.pose.position.x)
        self.map_y_ = float(msg.pose.pose.position.y)
        self.map_yaw_ = yaw_from_quat(
            float(msg.pose.pose.orientation.z),
            float(msg.pose.pose.orientation.w),
        )
        self.last_good_mx_ = self.map_x_
        self.last_good_my_ = self.map_y_

        src = self.filt_ if self.filt_ is not None else self.wheel_
        if src is not None:
            self.odom_x0_ = float(src.pose.pose.position.x)
            self.odom_y0_ = float(src.pose.pose.position.y)
            self.odom_yaw0_ = yaw_from_quat(
                float(src.pose.pose.orientation.z),
                float(src.pose.pose.orientation.w),
            )
        else:
            self.odom_x0_ = 0.0
            self.odom_y0_ = 0.0
            self.odom_yaw0_ = 0.0

        self.imu_yaw0_ = (
            self.imu_yaw_ if self.imu_yaw_ is not None else self.map_yaw_
        )
        self.slip_count_ = 0
        self.slipping_ = False
        self.seeded_ = True
        self.get_logger().info(
            "Set pose x=%.3f y=%.3f yaw=%.3f"
            % (self.map_x_, self.map_y_, self.map_yaw_)
        )

    def _on_filtered(self, msg: Odometry) -> None:
        self.filt_ = msg

    def _on_wheel(self, msg: Odometry) -> None:
        self.wheel_ = msg

    def _on_imu(self, msg: Imu) -> None:
        self.imu_yaw_ = yaw_from_quat(
            float(msg.orientation.z), float(msg.orientation.w)
        )
        self.ax_ = float(msg.linear_acceleration.x)
        self.ay_ = float(msg.linear_acceleration.y)

    def _detect_slip(self) -> bool:
        if self.wheel_ is None:
            return False
        vx = abs(float(self.wheel_.twist.twist.linear.x))
        vy = abs(float(self.wheel_.twist.twist.linear.y))
        wheel_speed = math.hypot(vx, vy)
        a_horiz = math.hypot(self.ax_, self.ay_)
        if wheel_speed < self.slip_vx_:
            self.slip_count_ = 0
            return False
        if a_horiz < self.slip_accel_:
            self.slip_count_ += 1
        else:
            self.slip_count_ = 0
        return self.slip_count_ >= self.slip_hold_

    def _tick(self) -> None:
        if not self.seeded_:
            return
        src = self.filt_ if self.filt_ is not None else self.wheel_
        if src is None:
            return

        slipping = self._detect_slip()
        if slipping and not self.slipping_:
            self.get_logger().warn("Slip detected — freezing XY")
        elif not slipping and self.slipping_:
            self.get_logger().info("Slip cleared — resuming XY")
            self.odom_x0_ = float(src.pose.pose.position.x)
            self.odom_y0_ = float(src.pose.pose.position.y)
            self.map_x_ = self.last_good_mx_
            self.map_y_ = self.last_good_my_
        self.slipping_ = slipping

        ox = float(src.pose.pose.position.x)
        oy = float(src.pose.pose.position.y)
        o_yaw = yaw_from_quat(
            float(src.pose.pose.orientation.z),
            float(src.pose.pose.orientation.w),
        )
        dx = ox - self.odom_x0_
        dy = oy - self.odom_y0_
        c = math.cos(self.map_yaw_)
        s = math.sin(self.map_yaw_)

        if self.slipping_:
            mx = self.last_good_mx_
            my = self.last_good_my_
        else:
            mx = self.map_x_ + c * dx - s * dy
            my = self.map_y_ + s * dx + c * dy
            self.last_good_mx_ = mx
            self.last_good_my_ = my

        if self.imu_yaw_ is not None:
            myaw = normalize_angle(self.map_yaw_ + (self.imu_yaw_ - self.imu_yaw0_))
        else:
            myaw = normalize_angle(self.map_yaw_ + (o_yaw - self.odom_yaw0_))

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.map_frame_
        pose.pose.position.x = mx
        pose.pose.position.y = my
        z, w = quat_from_yaw(myaw)
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        self.pose_pub_.publish(pose)

        od = Odometry()
        od.header = pose.header
        od.child_frame_id = self.base_frame_
        od.pose.pose = pose.pose
        if self.wheel_ is not None and not self.slipping_:
            od.twist = self.wheel_.twist
        self.odom_pub_.publish(od)

        # Publish map → odom (EKF owns odom → base_footprint). Avoids dual parents.
        yaw_mo = normalize_angle(myaw - o_yaw)
        c = math.cos(yaw_mo)
        s = math.sin(yaw_mo)
        tx = mx - (c * ox - s * oy)
        ty = my - (s * ox + c * oy)
        qz, qw = quat_from_yaw(yaw_mo)

        tf = TransformStamped()
        tf.header.stamp = pose.header.stamp
        tf.header.frame_id = self.map_frame_
        tf.child_frame_id = self.odom_frame_
        tf.transform.translation.x = tx
        tf.transform.translation.y = ty
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster_.sendTransform(tf)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LocalizationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
