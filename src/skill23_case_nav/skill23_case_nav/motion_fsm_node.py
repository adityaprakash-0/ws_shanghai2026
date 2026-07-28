#!/usr/bin/env python3
"""Motion FSM — exact port of src/motion_fsm_node.cpp."""

from __future__ import annotations

import json
import math
from enum import Enum, auto
from typing import Any, Callable, Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger

from skill23_case_nav.utils import normalize_angle, yaw_from_quat


class InternalState(Enum):
    IDLE = auto()
    EXEC_DRIVE = auto()
    EXEC_TURN = auto()
    CORRECT_DRIVE = auto()
    CORRECT_TURN = auto()
    STEP_PAUSE = auto()
    RECOVERY = auto()
    SPIN_360 = auto()
    RECOVERY_TURN = auto()
    WAIT_LIDAR_IDLE = auto()


class MotionFsmNode(Node):
    def __init__(self) -> None:
        super().__init__("cn_motion_fsm")

        self.declare_parameter("forward_speed", 0.15)
        self.declare_parameter("turn_speed", 0.30)
        self.declare_parameter("spin_speed", 0.15)
        self.declare_parameter("center_kp", 1.2)
        self.declare_parameter("enable_wall_centering", False)
        self.declare_parameter("heading_kp", 0.8)
        self.declare_parameter("heading_max_corr", 0.15)
        self.declare_parameter("yaw_tol_deg", 3.0)
        self.declare_parameter("goal_xy_tol", 0.15)
        self.declare_parameter("max_recovery_attempts", 5)
        self.declare_parameter("lidar_fov_deg", 90.0)
        self.declare_parameter("front_block_m", 0.20)
        self.declare_parameter("drive_block_margin_m", 0.05)
        self.declare_parameter("lidar_idle_wait_sec", 3.0)
        self.declare_parameter("front_scan_topic", "/scan/front")
        self.declare_parameter("enable_step_correction", True)
        self.declare_parameter("step_corr_xy_m", 0.02)
        self.declare_parameter("step_corr_yaw_deg", 2.0)
        self.declare_parameter("corr_speed", 0.08)
        self.declare_parameter("corr_turn_speed", 0.20)
        self.declare_parameter("step_pause_sec", 1.0)

        self.forward_speed_ = float(self.get_parameter("forward_speed").value)
        self.turn_speed_ = float(self.get_parameter("turn_speed").value)
        self.spin_speed_ = float(self.get_parameter("spin_speed").value)
        self.center_kp_ = float(self.get_parameter("center_kp").value)
        self.enable_wall_centering_ = bool(
            self.get_parameter("enable_wall_centering").value
        )
        self.heading_kp_ = float(self.get_parameter("heading_kp").value)
        self.heading_max_corr_ = float(self.get_parameter("heading_max_corr").value)
        self.yaw_tol_ = float(self.get_parameter("yaw_tol_deg").value) * math.pi / 180.0
        self.goal_xy_tol_ = float(self.get_parameter("goal_xy_tol").value)
        self.max_recovery_ = int(self.get_parameter("max_recovery_attempts").value)
        self.lidar_half_fov_ = (
            float(self.get_parameter("lidar_fov_deg").value) * math.pi / 360.0
        )
        self.front_block_m_ = float(self.get_parameter("front_block_m").value)
        self.drive_block_margin_ = float(
            self.get_parameter("drive_block_margin_m").value
        )
        self.lidar_idle_wait_sec_ = float(
            self.get_parameter("lidar_idle_wait_sec").value
        )
        self.enable_step_corr_ = bool(
            self.get_parameter("enable_step_correction").value
        )
        self.step_corr_xy_ = float(self.get_parameter("step_corr_xy_m").value)
        self.step_corr_yaw_ = (
            float(self.get_parameter("step_corr_yaw_deg").value) * math.pi / 180.0
        )
        self.corr_speed_ = float(self.get_parameter("corr_speed").value)
        self.corr_turn_speed_ = float(self.get_parameter("corr_turn_speed").value)
        self.step_pause_sec_ = float(self.get_parameter("step_pause_sec").value)
        front_scan_topic = str(self.get_parameter("front_scan_topic").value)

        self.state_ = InternalState.IDLE
        self.decision_ = "Idle"
        self.pose_: Optional[PoseStamped] = None
        self.watch_: Dict[str, Any] = {}
        self.lidar_status_: Dict[str, Any] = {}
        self.primitives_: List[Dict[str, Any]] = []
        self.idx_ = 0
        self.estop_ = False
        self.active_ = False

        self.drive_start_xy_: Optional[Tuple[float, float]] = None
        self.drive_target_dist_ = 0.0
        self.hold_yaw_ = 0.0
        self.locked_heading_: Optional[float] = None  # set after turns; used by next DRIVE

        self.turn_target_: Optional[float] = None
        self.turn_start_yaw_ = 0.0
        self.turn_target_delta_ = math.pi / 2.0
        self.pending_turn_ = ""

        self.recovery_attempts_ = 0
        self.recovery_phase_ = 0
        self.recovery_t0_ = 0.0
        self.saw_lidar_awake_ = False
        self.pause_t0_ = 0.0

        self.spin_start_yaw_ = 0.0
        self.spin_accum_ = 0.0
        self.last_spin_yaw_ = 0.0

        self.last_scan_: Optional[LaserScan] = None
        self.front_range_m_: Optional[float] = None

        self.cmd_pub_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.state_pub_ = self.create_publisher(String, "/case_nav/fsm_state", 10)
        self.replan_pub_ = self.create_publisher(String, "/case_nav/replan", 10)
        self.lidar_wake_pub_ = self.create_publisher(String, "/case_nav/lidar_wake", 10)

        self.map_reset_client_ = self.create_client(Trigger, "/case_nav/map/reset")

        self.create_subscription(PoseStamped, "/case_nav/pose", self.on_pose, 10)
        self.create_subscription(String, "/case_nav/watchdog", self.on_watch, 10)
        self.create_subscription(String, "/case_nav/plan", self.on_plan, 10)
        self.create_subscription(String, "/case_nav/cmd", self.on_cmd, 10)
        self.create_subscription(
            String, "/case_nav/lidar_status", self.on_lidar_status, 10
        )
        self.create_subscription(
            LaserScan, front_scan_topic, self.on_scan, qos_profile_sensor_data
        )

        self.create_timer(0.05, self.tick)
        self.get_logger().info(
            "Motion FSM: front scan=%s FOV=±%.0f° block=%.2fm corr=%s"
            % (
                front_scan_topic,
                float(self.get_parameter("lidar_fov_deg").value) * 0.5,
                self.front_block_m_,
                self.enable_step_corr_,
            )
        )

    @staticmethod
    def yaw_of(pose: PoseStamped) -> float:
        return yaw_from_quat(pose.pose.orientation.z, pose.pose.orientation.w)

    def xy(self) -> Optional[Tuple[float, float]]:
        if self.pose_ is None:
            return None
        return (self.pose_.pose.position.x, self.pose_.pose.position.y)

    def dist_from(self, start: Tuple[float, float]) -> float:
        p = self.xy()
        if p is None:
            return 0.0
        return math.hypot(p[0] - start[0], p[1] - start[1])

    def stop(self) -> None:
        self.cmd_pub_.publish(Twist())

    def published_mode(self) -> str:
        if self.estop_:
            return "E_STOP"
        if self.pose_ is None:
            return "WAITING_POSE"
        if self.state_ in (
            InternalState.RECOVERY,
            InternalState.SPIN_360,
            InternalState.RECOVERY_TURN,
            InternalState.WAIT_LIDAR_IDLE,
        ):
            return "RECOVERY"
        if self.active_ and self.state_ == InternalState.IDLE and not self.primitives_:
            return "PLANNING"
        if self.active_ and (
            self.state_ != InternalState.IDLE or bool(self.primitives_)
        ):
            return "NAVIGATING"
        return "IDLE"

    def remaining_steps(self) -> List[Dict[str, Any]]:
        return list(self.primitives_[self.idx_ :])

    def step_statuses(self) -> List[str]:
        """done | active | pending for every planned step (kept for dashboard)."""
        out: List[str] = []
        for i in range(len(self.primitives_)):
            if i < self.idx_:
                out.append("done")
            elif i == self.idx_ and self.active_ and self.state_ not in (
                InternalState.IDLE,
            ):
                out.append("active")
            elif i == self.idx_ and self.active_:
                out.append("active")
            else:
                out.append("pending")
        if self.active_ and self.idx_ < len(self.primitives_) and self.state_ in (
            InternalState.EXEC_DRIVE,
            InternalState.EXEC_TURN,
            InternalState.CORRECT_DRIVE,
            InternalState.CORRECT_TURN,
            InternalState.STEP_PAUSE,
        ):
            # ensure current is active
            out = [
                "done" if i < self.idx_ else ("active" if i == self.idx_ else "pending")
                for i in range(len(self.primitives_))
            ]
        return out

    def step_progress(self) -> Dict[str, Any]:
        travelled = 0.0
        if self.drive_start_xy_ is not None:
            travelled = self.dist_from(self.drive_start_xy_)
        turn_deg = 0.0
        if self.pose_ is not None and self.state_ in (
            InternalState.EXEC_TURN,
            InternalState.CORRECT_TURN,
            InternalState.RECOVERY_TURN,
        ):
            turn_deg = abs(
                normalize_angle(self.yaw_of(self.pose_) - self.turn_start_yaw_)
            ) * 180.0 / math.pi
        return {
            "travelled_m": round(travelled, 3),
            "target_m": round(self.drive_target_dist_, 3),
            "turn_deg": round(turn_deg, 1),
            "turn_target_deg": round(abs(self.turn_target_delta_) * 180.0 / math.pi, 1),
            "kind": (
                "DRIVE"
                if self.state_ in (InternalState.EXEC_DRIVE, InternalState.CORRECT_DRIVE)
                else (
                    "TURN"
                    if self.state_
                    in (
                        InternalState.EXEC_TURN,
                        InternalState.CORRECT_TURN,
                        InternalState.RECOVERY_TURN,
                    )
                    else ""
                )
            ),
        }

    def publish_state(self) -> None:
        front: Any = None
        if self.front_range_m_ is not None:
            front = round(self.front_range_m_ * 1000.0) / 1000.0
        msg = String()
        msg.data = json.dumps(
            {
                "state": self.published_mode(),
                "decision": self.decision_,
                "active": self.active_,
                "estop": self.estop_,
                "primitive_idx": self.idx_,
                "primitives_left": max(0, len(self.primitives_) - self.idx_),
                "remaining_steps": self.remaining_steps(),
                "all_steps": self.primitives_,
                "step_statuses": self.step_statuses(),
                "step_progress": self.step_progress(),
                "pending_turn": self.pending_turn_,
                "recovery_attempts": self.recovery_attempts_,
                "front_range_m": front,
                "wall_centering": self.enable_wall_centering_,
            }
        )
        self.state_pub_.publish(msg)

    def set_decision(self, text: str) -> None:
        if text != self.decision_:
            self.decision_ = text
            self.get_logger().info(f"DECISION: {text}")

    def on_pose(self, msg: PoseStamped) -> None:
        self.pose_ = msg

    def on_watch(self, msg: String) -> None:
        try:
            self.watch_ = json.loads(msg.data)
        except Exception:
            pass

    def on_plan(self, msg: String) -> None:
        try:
            d = json.loads(msg.data)
            self.primitives_ = list(d.get("primitives", []))
            self.idx_ = 0
            if self.state_ not in (
                InternalState.RECOVERY,
                InternalState.SPIN_360,
                InternalState.RECOVERY_TURN,
                InternalState.WAIT_LIDAR_IDLE,
            ):
                self.recovery_attempts_ = 0
            if self.active_ and self.primitives_:
                self.start_current_primitive()
            elif self.active_ and not self.primitives_:
                self.state_ = InternalState.IDLE
                self.set_decision("Plan empty — waiting")
            self.get_logger().info(
                f"Got plan with {len(self.primitives_)} primitives"
            )
        except Exception as e:
            self.get_logger().warn(f"Bad plan: {e}")

    def on_cmd(self, msg: String) -> None:
        try:
            d = json.loads(msg.data)
        except Exception:
            d = {"cmd": msg.data}
        cmd = str(d.get("cmd", ""))
        lower = cmd.lower()
        if lower == "start":
            if self.pose_ is None:
                self.set_decision("Waiting for pose — set pose first")
                return
            self.estop_ = False
            self.active_ = True
            if self.primitives_:
                self.start_current_primitive()
            else:
                self.state_ = InternalState.IDLE
                self.set_decision("Started — waiting for plan")
        elif lower == "stop":
            self.active_ = False
            self.state_ = InternalState.IDLE
            self.stop()
            self.set_decision("Stopped")
        elif lower == "estop":
            self.estop_ = bool(d.get("active", True))
            if self.estop_:
                self.active_ = False
                self.state_ = InternalState.IDLE
                self.stop()
                self.set_decision("E-STOP")
            else:
                self.set_decision("Idle — waiting for Start")

    def on_lidar_status(self, msg: String) -> None:
        try:
            self.lidar_status_ = json.loads(msg.data)
        except Exception:
            pass

    def on_scan(self, msg: LaserScan) -> None:
        self.last_scan_ = msg
        self.front_range_m_ = self.min_front_range(msg)

    def min_front_range(self, scan: LaserScan) -> Optional[float]:
        hard_min = 0.02
        rmin = min(float(scan.range_min), hard_min) if scan.range_min > 0 else hard_min
        best: Optional[float] = None
        angle = float(scan.angle_min)
        for r in scan.ranges:
            if (
                abs(angle) <= self.lidar_half_fov_
                and math.isfinite(r)
                and r >= rmin
                and r < scan.range_max
            ):
                if best is None or r < best:
                    best = float(r)
            angle += scan.angle_increment
        return best

    def front_blocked(self, _lookahead_m: float = 0.0) -> bool:
        """Hard stop if anything is closer than front_block_m (lidar frame)."""
        # Own scan
        scan_hit = (
            self.front_range_m_ is not None
            and self.front_range_m_ < self.front_block_m_
        )
        # Watchdog (debounced) as backup
        wd_hit = bool(self.watch_.get("front_blocked", False))
        return scan_hit or wd_hit

    def start_current_primitive(self) -> None:
        if self.idx_ >= len(self.primitives_):
            self.state_ = InternalState.IDLE
            self.active_ = False
            self.stop()
            self.set_decision("Done — goal reached")
            self.get_logger().info("Mission complete")
            return
        prim = self.primitives_[self.idx_]
        ptype = str(prim.get("type", ""))
        if ptype == "DRIVE":
            self.state_ = InternalState.EXEC_DRIVE
            self.drive_start_xy_ = self.xy()
            self.drive_target_dist_ = float(prim.get("distance_m", 0.3))
            # After a turn, keep the locked heading; otherwise snap to current yaw
            if self.locked_heading_ is not None:
                self.hold_yaw_ = self.locked_heading_
            elif self.pose_ is not None:
                self.hold_yaw_ = self.yaw_of(self.pose_)
            else:
                self.hold_yaw_ = 0.0
            dist_s = f"{self.drive_target_dist_:.4f}"[:4]
            self.set_decision(f"Driving forward {dist_s} m (hold yaw)")
        elif ptype in ("TURN_LEFT_90", "TURN_RIGHT_90"):
            self.pending_turn_ = "LEFT" if ptype == "TURN_LEFT_90" else "RIGHT"
            self.locked_heading_ = None  # will lock when turn finishes
            self.set_decision(f"Turning {self.pending_turn_} 90°")
            self.begin_turn(self.pending_turn_)
        else:
            self.get_logger().warn(f"Unknown primitive {ptype}, skipping")
            self.idx_ += 1
            self.start_current_primitive()

    def begin_step_pause(self) -> None:
        """1 s dwell between completed steps before starting the next."""
        self.stop()
        if self.idx_ + 1 >= len(self.primitives_):
            # last step done — no pause needed before "mission complete"
            self.idx_ += 1
            self.pending_turn_ = ""
            self.start_current_primitive()
            return
        self.state_ = InternalState.STEP_PAUSE
        self.pause_t0_ = self.get_clock().now().nanoseconds * 1e-9
        self.set_decision(f"Pause {self.step_pause_sec_:.1f}s before next step")

    def advance_after_step(self) -> None:
        self.pending_turn_ = ""
        self.begin_step_pause()

    def tick_step_pause(self) -> None:
        self.stop()
        t = self.get_clock().now().nanoseconds * 1e-9
        left = self.step_pause_sec_ - (t - self.pause_t0_)
        if left <= 0.0:
            self.idx_ += 1
            self.start_current_primitive()
            return
        self.set_decision(f"Pause · {left:.1f}s left")

    def finish_drive_step(self) -> None:
        self.stop()
        travelled = (
            self.dist_from(self.drive_start_xy_)
            if self.drive_start_xy_ is not None
            else self.drive_target_dist_
        )
        err = travelled - self.drive_target_dist_
        if self.enable_step_corr_ and abs(err) > self.step_corr_xy_:
            self.state_ = InternalState.CORRECT_DRIVE
            self.set_decision(f"Correct drive · err {err:+.3f} m")
            return
        self.advance_after_step()

    def finish_turn_step(
        self, consume: bool, after: Optional[Callable[[], None]]
    ) -> None:
        self.stop()
        if not consume:
            self.turn_target_ = None
            if after is not None:
                after()
            return
        if self.pose_ is None or self.turn_target_ is None:
            self.turn_target_ = None
            self.advance_after_step()
            return
        err = normalize_angle(self.turn_target_ - self.yaw_of(self.pose_))
        if self.enable_step_corr_ and abs(err) > self.step_corr_yaw_:
            self.state_ = InternalState.CORRECT_TURN
            self.set_decision(
                f"Correct turn · err {err * 180.0 / math.pi:+.1f}°"
            )
            return
        # Lock heading for the next forward drive
        if self.turn_target_ is not None:
            self.locked_heading_ = self.turn_target_
        elif self.pose_ is not None:
            self.locked_heading_ = self.yaw_of(self.pose_)
        self.turn_target_ = None
        self.advance_after_step()

    def tick_correct_drive(self) -> None:
        if self.drive_start_xy_ is None or self.pose_ is None:
            self.advance_after_step()
            return
        travelled = self.dist_from(self.drive_start_xy_)
        err = travelled - self.drive_target_dist_
        if abs(err) <= self.step_corr_xy_:
            self.set_decision(
                f"Drive corrected · {travelled:.3f}/{self.drive_target_dist_:.3f} m"
            )
            self.advance_after_step()
            return
        cmd = Twist()
        cmd.linear.x = -self.corr_speed_ if err > 0.0 else self.corr_speed_
        # Keep heading while correcting distance
        if self.pose_ is not None:
            yaw_err = normalize_angle(self.hold_yaw_ - self.yaw_of(self.pose_))
            cmd.angular.z = max(
                -self.heading_max_corr_,
                min(self.heading_max_corr_, self.heading_kp_ * yaw_err),
            )
        self.cmd_pub_.publish(cmd)
        self.set_decision(
            f"Correcting drive · {travelled:.3f}/{self.drive_target_dist_:.3f} m"
        )

    def tick_correct_turn(self) -> None:
        if self.turn_target_ is None or self.pose_ is None:
            self.advance_after_step()
            return
        err = normalize_angle(self.turn_target_ - self.yaw_of(self.pose_))
        if abs(err) <= self.step_corr_yaw_:
            self.locked_heading_ = self.turn_target_
            self.turn_target_ = None
            self.set_decision("Turn corrected")
            self.advance_after_step()
            return
        cmd = Twist()
        cmd.angular.z = (
            self.corr_turn_speed_ if err > 0.0 else -self.corr_turn_speed_
        )
        self.cmd_pub_.publish(cmd)
        self.set_decision(
            f"Correcting turn · err {err * 180.0 / math.pi:+.1f}°"
        )

    def drive_cmd(
        self, center: bool, speed: Optional[float] = None
    ) -> Twist:
        cmd = Twist()
        cmd.linear.x = self.forward_speed_ if speed is None else speed
        wz = 0.0
        if self.pose_ is not None:
            err = normalize_angle(self.hold_yaw_ - self.yaw_of(self.pose_))
            wz += max(
                -self.heading_max_corr_,
                min(self.heading_max_corr_, self.heading_kp_ * err),
            )
        if center and self.enable_wall_centering_:
            us_l_raw = self.watch_.get("us_left_m", None)
            us_r_raw = self.watch_.get("us_right_m", None)
            us_l = -1.0 if us_l_raw is None else float(us_l_raw)
            us_r = -1.0 if us_r_raw is None else float(us_r_raw)
            wall_l = bool(self.watch_.get("wall_left", False))
            wall_r = bool(self.watch_.get("wall_right", False))
            if wall_l and wall_r and us_l >= 0.0 and us_r >= 0.0:
                wz += max(-0.4, min(0.4, self.center_kp_ * (us_r - us_l)))
            elif wall_l and not wall_r:
                wz -= 0.08
            elif wall_r and not wall_l:
                wz += 0.08
        cmd.angular.z = wz
        return cmd

    def begin_turn(self, direction: str) -> None:
        if self.pose_ is None:
            return
        yaw = self.yaw_of(self.pose_)
        delta = math.pi / 2.0 if direction == "LEFT" else -math.pi / 2.0
        self.turn_start_yaw_ = yaw
        self.turn_target_delta_ = delta
        self.turn_target_ = normalize_angle(yaw + delta)
        self.state_ = InternalState.EXEC_TURN
        self.set_decision(f"Turning {direction} 90°")

    def begin_spin_360(self) -> None:
        if self.pose_ is None:
            self.state_ = InternalState.RECOVERY
            self.recovery_phase_ = 2
            return
        self.spin_start_yaw_ = self.yaw_of(self.pose_)
        self.spin_accum_ = 0.0
        self.last_spin_yaw_ = self.spin_start_yaw_
        self.state_ = InternalState.SPIN_360
        self.set_decision("Spinning 360°")

    def next_is_turn(self) -> bool:
        nxt = self.idx_ + 1
        if nxt >= len(self.primitives_):
            return False
        ptype = str(self.primitives_[nxt].get("type", ""))
        return ptype.startswith("TURN_")

    def drive_done(self) -> bool:
        if self.drive_start_xy_ is None:
            return False
        return self.dist_from(self.drive_start_xy_) >= self.drive_target_dist_

    def enter_recovery(self) -> None:
        self.recovery_attempts_ += 1
        if self.recovery_attempts_ > self.max_recovery_:
            self.get_logger().error("Max recovery attempts — stopping")
            self.active_ = False
            self.state_ = InternalState.IDLE
            self.stop()
            self.set_decision("Stopped — recovery failed")
            return
        self.state_ = InternalState.RECOVERY
        self.recovery_phase_ = 0
        self.recovery_t0_ = self.get_clock().now().nanoseconds * 1e-9
        self.stop()
        self.set_decision(
            f"Blocked ahead — recovery #{self.recovery_attempts_}"
        )

    def call_map_reset(self) -> None:
        if not self.map_reset_client_.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn("Map reset service unavailable")
            return
        self.map_reset_client_.call_async(Trigger.Request())

    def wake_lidar(self, duration_sec: float = 30.0) -> None:
        wake = String()
        wake.data = json.dumps({"wake": True, "duration_sec": duration_sec})
        self.lidar_wake_pub_.publish(wake)

    def request_replan(self) -> None:
        self.replan_pub_.publish(String())
        self.active_ = True
        self.state_ = InternalState.IDLE
        self.primitives_ = []
        self.idx_ = 0
        self.stop()
        self.set_decision("Replanning on updated map")

    def tick(self) -> None:
        self.publish_state()
        if self.estop_:
            self.stop()
            self.set_decision("E-STOP")
            return
        if not self.active_:
            if self.state_ == InternalState.IDLE and self.decision_ not in (
                "Done — goal reached",
                "Stopped",
                "Stopped — recovery failed",
            ):
                if self.pose_ is None:
                    self.set_decision("Waiting for pose")
                else:
                    self.set_decision("Idle — waiting for Start")
            return
        if self.pose_ is None:
            self.set_decision("Waiting for pose")
            return

        if self.state_ == InternalState.EXEC_DRIVE:
            done = self.drive_done()
            # Always hard-stop on front obstacle while driving (no early-turn skip).
            if self.front_blocked():
                self.stop()
                self.set_decision(
                    "Front blocked @ %.2fm — stop"
                    % (
                        self.front_range_m_
                        if self.front_range_m_ is not None
                        else -1.0
                    )
                )
                self.enter_recovery()
                return
            if done:
                self.finish_drive_step()
                return

        if self.state_ == InternalState.IDLE:
            self.stop()
        elif self.state_ == InternalState.EXEC_DRIVE:
            self.tick_drive()
        elif self.state_ == InternalState.EXEC_TURN:
            self.tick_turn(True, None)
        elif self.state_ == InternalState.CORRECT_DRIVE:
            self.tick_correct_drive()
        elif self.state_ == InternalState.CORRECT_TURN:
            self.tick_correct_turn()
        elif self.state_ == InternalState.STEP_PAUSE:
            self.tick_step_pause()
        elif self.state_ == InternalState.SPIN_360:
            self.tick_spin_360()
        elif self.state_ == InternalState.RECOVERY_TURN:
            self.tick_turn(False, self.request_replan)
        elif self.state_ == InternalState.WAIT_LIDAR_IDLE:
            self.tick_wait_lidar_idle()
        elif self.state_ == InternalState.RECOVERY:
            self.tick_recovery()

    def begin_recovery_turn_180(self) -> None:
        if self.pose_ is None:
            return
        self.turn_target_ = normalize_angle(self.yaw_of(self.pose_) + math.pi)
        self.turn_start_yaw_ = self.yaw_of(self.pose_)
        self.turn_target_delta_ = math.pi
        self.state_ = InternalState.RECOVERY_TURN
        self.set_decision("Spinning 180° — all sides blocked")

    def begin_recovery_turn_open(self) -> None:
        open_l = bool(self.watch_.get("open_left", False))
        open_r = bool(self.watch_.get("open_right", False))
        wall_l = bool(self.watch_.get("wall_left", False))
        wall_r = bool(self.watch_.get("wall_right", False))
        front_blk = self.front_blocked(self.front_block_m_)

        if front_blk and wall_l and wall_r:
            self.begin_recovery_turn_180()
            return
        if open_l and not open_r:
            self.begin_turn("LEFT")
            self.state_ = InternalState.RECOVERY_TURN
            self.set_decision("Recovery turn LEFT toward open side")
        elif open_r and not open_l:
            self.begin_turn("RIGHT")
            self.state_ = InternalState.RECOVERY_TURN
            self.set_decision("Recovery turn RIGHT toward open side")
        elif open_l:
            self.begin_turn("LEFT")
            self.state_ = InternalState.RECOVERY_TURN
            self.set_decision("Recovery turn LEFT toward open side")
        elif open_r:
            self.begin_turn("RIGHT")
            self.state_ = InternalState.RECOVERY_TURN
            self.set_decision("Recovery turn RIGHT toward open side")
        else:
            self.begin_recovery_turn_180()

    def begin_wait_lidar_idle(self) -> None:
        self.state_ = InternalState.WAIT_LIDAR_IDLE
        self.recovery_t0_ = self.get_clock().now().nanoseconds * 1e-9
        self.saw_lidar_awake_ = False
        self.stop()
        self.set_decision("Lidar updating map")

    def tick_drive(self) -> None:
        if self.drive_start_xy_ is None:
            self.drive_start_xy_ = self.xy()
        travelled = (
            self.dist_from(self.drive_start_xy_)
            if self.drive_start_xy_ is not None
            else 0.0
        )
        remain = max(0.0, self.drive_target_dist_ - travelled)
        self.set_decision(
            f"Driving · {travelled:.3f}/{self.drive_target_dist_:.3f} m"
            f" ({remain:.3f} left)"
        )
        if travelled >= self.drive_target_dist_:
            self.finish_drive_step()
            return
        self.cmd_pub_.publish(self.drive_cmd(True, None))

    def tick_turn(
        self, consume: bool, after: Optional[Callable[[], None]]
    ) -> None:
        if self.turn_target_ is None or self.pose_ is None:
            self.stop()
            return
        err = normalize_angle(self.turn_target_ - self.yaw_of(self.pose_))
        turned = abs(
            normalize_angle(self.yaw_of(self.pose_) - self.turn_start_yaw_)
        ) * 180.0 / math.pi
        tgt = abs(self.turn_target_delta_) * 180.0 / math.pi
        self.set_decision(f"Turning · {turned:.1f}/{tgt:.0f}°")
        if abs(err) <= self.yaw_tol_:
            self.finish_turn_step(consume, after)
            return
        cmd = Twist()
        cmd.angular.z = self.turn_speed_ if err > 0.0 else -self.turn_speed_
        if abs(err) < math.pi / 9.0:
            cmd.angular.z *= 0.5
        self.cmd_pub_.publish(cmd)

    def tick_spin_360(self) -> None:
        if self.pose_ is None:
            return
        yaw = self.yaw_of(self.pose_)
        delta = normalize_angle(yaw - self.last_spin_yaw_)
        self.spin_accum_ += abs(delta)
        self.last_spin_yaw_ = yaw

        if self.spin_accum_ >= 2.0 * math.pi - self.yaw_tol_:
            self.stop()
            self.recovery_phase_ = 2
            self.state_ = InternalState.RECOVERY
            return
        cmd = Twist()
        cmd.angular.z = self.spin_speed_
        self.cmd_pub_.publish(cmd)
        deg = int(self.spin_accum_ * 180.0 / math.pi)
        self.set_decision(f"Spinning 360° · {deg}°")

    def tick_recovery(self) -> None:
        t = self.get_clock().now().nanoseconds * 1e-9
        if self.recovery_phase_ == 0:
            self.stop()
            self.recovery_phase_ = 1
            self.begin_spin_360()
        elif self.recovery_phase_ == 2:
            self.stop()
            self.call_map_reset()
            self.wake_lidar(30.0)
            self.recovery_phase_ = 3
            self.recovery_t0_ = t
            self.set_decision("Map reset + waking lidar")
        elif self.recovery_phase_ == 3:
            if t - self.recovery_t0_ >= 0.4:
                self.recovery_phase_ = 4
                self.begin_wait_lidar_idle()

    def tick_wait_lidar_idle(self) -> None:
        t = self.get_clock().now().nanoseconds * 1e-9
        awake = bool(self.lidar_status_.get("awake", False))
        if awake:
            self.saw_lidar_awake_ = True
            self.recovery_t0_ = t
            self.set_decision("Lidar updating map")
            self.stop()
            return
        wait = 0.3 if self.saw_lidar_awake_ else self.lidar_idle_wait_sec_
        if t - self.recovery_t0_ >= wait:
            self.begin_recovery_turn_open()
            return
        self.set_decision(
            "Lidar idle · turning toward open"
            if self.saw_lidar_awake_
            else "Waiting for lidar stamps"
        )
        self.stop()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = MotionFsmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
