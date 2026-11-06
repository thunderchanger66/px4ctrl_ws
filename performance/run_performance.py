#!/usr/bin/env python3
"""Run a bounded PX4Ctrl SITL waypoint suite and record synchronized state."""

import argparse
import csv
import json
import math
import os
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import (
    FailsafeFlags,
    VehicleAttitude,
    VehicleAttitudeSetpoint,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)
from quadrotor_msgs.msg import PositionCommand
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, String


FIELDS = [
    "elapsed_s", "wall_time_unix", "goal_index", "goal_x", "goal_y", "goal_z",
    "cmd_x", "cmd_y", "cmd_z", "cmd_vx", "cmd_vy", "cmd_vz", "cmd_yaw",
    "odom_x", "odom_y", "odom_z", "odom_vx_body", "odom_vy_body", "odom_vz_body",
    "px4_x", "px4_y", "px4_z", "px4_vx", "px4_vy", "px4_vz", "px4_heading",
    "xy_reset", "z_reset", "heading_reset", "xy_valid", "z_valid", "dead_reckoning",
    "nav_state", "arming_state", "failsafe", "offboard_lost", "landed", "maybe_landed",
    "att_q_w", "att_q_x", "att_q_y", "att_q_z", "sp_q_w", "sp_q_x", "sp_q_y", "sp_q_z",
    "thrust_x", "thrust_y", "thrust_z", "yaw_sp_move_rate", "controller_status",
    "planner_emergency_count",
]


class PerformanceRun(Node):
    def __init__(self, args):
        super().__init__("px4ctrl_performance_run")
        self.args = args
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=30,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.command = None
        self.odom = None
        self.px4 = None
        self.status = None
        self.attitude = None
        self.setpoint = None
        self.failsafe = None
        self.land = None
        self.controller = ""
        self.emergencies = 0
        self.reset_signature = None
        self.reset_stable_since = None
        self.latest_odom_time = None
        self.latest_px4_time = None
        self.goal_index = -1
        self.active_goal = None
        self.started = time.monotonic()
        self.last_sample = 0.0
        self.writer = None
        self.stream = None
        self.events = []
        self.create_subscription(PositionCommand, "/drone_0_planning/pos_cmd", self.command_cb, 20)
        self.create_subscription(Odometry, "/fastlio2/lio_odom", self.odom_cb, 20)
        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self.px4_cb, qos)
        self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status_v4", self.status_cb, qos)
        self.create_subscription(VehicleAttitude, "/fmu/out/vehicle_attitude", self.attitude_cb, qos)
        self.create_subscription(VehicleAttitudeSetpoint, "/fmu/in/vehicle_attitude_setpoint_v1", self.sp_cb, qos)
        self.create_subscription(FailsafeFlags, "/fmu/out/failsafe_flags", self.failsafe_cb, qos)
        self.create_subscription(VehicleLandDetected, "/fmu/out/vehicle_land_detected", self.land_cb, qos)
        self.create_subscription(String, "/px4ctrl/status", self.controller_cb, 10)
        self.create_subscription(Empty, "/planning/emergency_stop", self.emergency_cb, 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.clear_yaw_pub = self.create_publisher(Empty, "/px4ctrl/clear_manual_yaw", 10)
        self.command_pub = self.create_publisher(PositionCommand, "/drone_0_planning/pos_cmd", 10)

    def command_cb(self, msg):
        if msg.trajectory_flag == PositionCommand.TRAJECTORY_STATUS_READY:
            self.command = msg

    def odom_cb(self, msg):
        if msg.header.frame_id == "lidar":
            self.odom = msg
            self.latest_odom_time = time.monotonic()

    def px4_cb(self, msg):
        sig = (msg.xy_reset_counter, msg.z_reset_counter, msg.heading_reset_counter)
        now = time.monotonic()
        if sig != self.reset_signature:
            self.reset_signature = sig
            self.reset_stable_since = now
        self.px4 = msg
        self.latest_px4_time = now

    def status_cb(self, msg): self.status = msg
    def attitude_cb(self, msg): self.attitude = msg
    def sp_cb(self, msg): self.setpoint = msg
    def failsafe_cb(self, msg): self.failsafe = msg
    def land_cb(self, msg): self.land = msg
    def controller_cb(self, msg): self.controller = msg.data
    def emergency_cb(self, _msg): self.emergencies += 1

    def spin_for(self, seconds):
        until = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < until:
            rclpy.spin_once(self, timeout_sec=min(0.05, max(0.0, until - time.monotonic())))
            self.sample()

    def sample(self):
        now = time.monotonic()
        if now - self.last_sample < 0.05 or self.odom is None or self.px4 is None:
            return
        self.last_sample = now
        o, p, c = self.odom, self.px4, self.command
        a, sp = self.attitude, self.setpoint
        g = self.active_goal or (math.nan, math.nan, math.nan)
        def val(obj, attr, default=math.nan):
            return getattr(obj, attr, default) if obj else default
        row = {
            "elapsed_s": now - self.started, "wall_time_unix": time.time(),
            "goal_index": self.goal_index, "goal_x": g[0], "goal_y": g[1], "goal_z": g[2],
            "cmd_x": val(getattr(c, "position", None), "x"),
            "cmd_y": val(getattr(c, "position", None), "y"),
            "cmd_z": val(getattr(c, "position", None), "z"),
            "cmd_vx": val(getattr(c, "velocity", None), "x"),
            "cmd_vy": val(getattr(c, "velocity", None), "y"),
            "cmd_vz": val(getattr(c, "velocity", None), "z"),
            "cmd_yaw": val(c, "yaw"),
            "odom_x": o.pose.pose.position.x, "odom_y": o.pose.pose.position.y,
            "odom_z": o.pose.pose.position.z,
            "odom_vx_body": o.twist.twist.linear.x, "odom_vy_body": o.twist.twist.linear.y,
            "odom_vz_body": o.twist.twist.linear.z,
            "px4_x": p.x, "px4_y": p.y, "px4_z": p.z,
            "px4_vx": p.vx, "px4_vy": p.vy, "px4_vz": p.vz, "px4_heading": p.heading,
            "xy_reset": p.xy_reset_counter, "z_reset": p.z_reset_counter,
            "heading_reset": p.heading_reset_counter,
            "xy_valid": int(p.xy_valid), "z_valid": int(p.z_valid),
            "dead_reckoning": int(p.dead_reckoning),
            "nav_state": val(self.status, "nav_state", -1),
            "arming_state": val(self.status, "arming_state", -1),
            "failsafe": int(val(self.status, "failsafe", False)),
            "offboard_lost": int(val(self.failsafe, "offboard_control_signal_lost", False)),
            "landed": int(val(self.land, "landed", False)),
            "maybe_landed": int(val(self.land, "maybe_landed", False)),
            "controller_status": self.controller, "planner_emergency_count": self.emergencies,
        }
        for prefix, msg, name in (("att", a, "q"), ("sp", sp, "q_d")):
            q = getattr(msg, name, []) if msg else []
            for i, axis in enumerate(("w", "x", "y", "z")):
                row[f"{prefix}_q_{axis}"] = q[i] if len(q) > i else math.nan
        thrust = sp.thrust_body if sp else []
        for i, axis in enumerate(("x", "y", "z")):
            row[f"thrust_{axis}"] = thrust[i] if len(thrust) > i else math.nan
        row["yaw_sp_move_rate"] = sp.yaw_sp_move_rate if sp else math.nan
        self.writer.writerow(row)
        if int((now - self.started) * 2) != int((self.last_sample - self.started - 0.05) * 2):
            self.stream.flush()

    def wait_ready(self, timeout):
        deadline = time.monotonic() + timeout
        hold = PositionCommand()
        hold.header.frame_id = "lidar"
        hold.trajectory_flag = PositionCommand.TRAJECTORY_STATUS_READY
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            self.sample()
            if self.odom and self.command_pub.get_subscription_count() > 0 and not self.controller.startswith(("takeoff_", "hover_", "tracking_")):
                hold.header.stamp = self.get_clock().now().to_msg()
                hold.position = self.command.position if self.command else self.odom.pose.pose.position
                self.command_pub.publish(hold)
            ready = (
                self.controller.startswith(("hover_", "tracking_"))
                and self.status and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
                and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
                and self.px4 and self.px4.xy_valid and self.px4.z_valid
                and self.px4.v_xy_valid and self.px4.v_z_valid and not self.px4.dead_reckoning
                and self.latest_px4_time and time.monotonic() - self.latest_px4_time < 0.5
                and self.reset_stable_since and time.monotonic() - self.reset_stable_since >= 2.0
                and math.sqrt(self.px4.vx**2 + self.px4.vy**2 + self.px4.vz**2) < 0.20
                and self.odom and self.latest_odom_time and time.monotonic() - self.latest_odom_time < 0.5
                and math.sqrt(self.odom.twist.twist.linear.x**2 + self.odom.twist.twist.linear.y**2 + self.odom.twist.twist.linear.z**2) < 0.20
            )
            if ready:
                return
            if self.controller.startswith("safety_fault"):
                raise RuntimeError(f"PX4Ctrl safety fault before mission: {self.controller}")
        raise TimeoutError(f"PX4Ctrl did not become ready; last status={self.controller!r}")

    def send_goal(self, index, target, timeout):
        self.goal_index, self.active_goal = index, target
        msg = PoseStamped()
        msg.header.frame_id = "lidar"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = target
        msg.pose.orientation.w = 1.0
        start = time.monotonic()
        start_pos = (self.odom.pose.pose.position.x, self.odom.pose.pose.position.y, self.odom.pose.pose.position.z)
        event = {"index": index, "target_lidar": target, "start_lidar": start_pos,
                 "sent_elapsed_s": start - self.started, "sent_wall_time_unix": time.time(), "result": "timeout"}
        self.events.append(event)
        for _ in range(3):
            self.clear_yaw_pub.publish(Empty())
            msg.header.stamp = self.get_clock().now().to_msg()
            self.goal_pub.publish(msg)
            self.spin_for(0.10)
        within_since = None
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            self.spin_for(0.05)
            if not self.status or self.status.failsafe:
                raise RuntimeError("PX4 reports failsafe during goal tracking")
            if self.status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                raise RuntimeError("PX4 disarmed during goal tracking")
            if self.status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                raise RuntimeError("PX4 left Offboard during goal tracking")
            if self.failsafe and self.failsafe.offboard_control_signal_lost:
                raise RuntimeError("PX4 reports Offboard control signal lost")
            if self.land and self.land.landed:
                raise RuntimeError("PX4 reports landed before reaching the goal")
            if self.odom:
                pos = self.odom.pose.pose.position
                speed = math.sqrt(sum(v*v for v in (
                    self.odom.twist.twist.linear.x, self.odom.twist.twist.linear.y,
                    self.odom.twist.twist.linear.z)))
                error = math.dist((pos.x, pos.y, pos.z), target)
                if error <= self.args.goal_tolerance and speed <= self.args.speed_tolerance:
                    within_since = within_since or time.monotonic()
                    if time.monotonic() - within_since >= self.args.settle_sec:
                        event.update({"result": "reached", "done_elapsed_s": time.monotonic() - self.started,
                                      "done_wall_time_unix": time.time(), "final_error_m": error,
                                      "final_speed_mps": speed})
                        return
                else:
                    within_since = None
        pos = self.odom.pose.pose.position if self.odom else None
        event["last_position_lidar"] = [pos.x, pos.y, pos.z] if pos else None
        raise TimeoutError(f"Goal {index} not reached within {timeout:.0f}s")

    def close(self):
        if self.stream:
            self.stream.flush()
            self.stream.close()
        with open(self.args.events, "w", encoding="utf-8") as f:
            json.dump({"frame": "lidar", "goal_tolerance_m": self.args.goal_tolerance,
                       "speed_tolerance_mps": self.args.speed_tolerance,
                       "settle_sec": self.args.settle_sec, "goals": self.events}, f, indent=2)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--csv", required=True)
    p.add_argument("--events", required=True)
    p.add_argument("--timeout", type=float, default=120.0)
    p.add_argument("--goal-tolerance", type=float, default=0.25)
    p.add_argument("--speed-tolerance", type=float, default=0.20)
    p.add_argument("--settle-sec", type=float, default=2.0)
    p.add_argument("--hold-after", type=float, default=5.0)
    args = p.parse_args()
    os.makedirs(os.path.dirname(os.path.abspath(args.csv)), exist_ok=True)
    rclpy.init()
    node = PerformanceRun(args)
    ok = False
    try:
        node.stream = open(args.csv, "w", newline="", encoding="utf-8")
        node.writer = csv.DictWriter(node.stream, fieldnames=FIELDS)
        node.writer.writeheader()
        node.stream.flush()
        node.wait_ready(args.timeout)
        node.events.append({"name": "takeoff_and_hover_baseline", "ready_elapsed_s": time.monotonic() - node.started,
                            "ready_wall_time_unix": time.time(), "controller_status": node.controller})
        goals = [(4.0, -4.0, 0.5), (0.0, 0.0, 0.5), (4.0, 0.0, 0.5), (0.0, 0.0, 0.5)]
        for i, goal in enumerate(goals, 1):
            print(f"Goal {i}/{len(goals)}: {goal} lidar, auto-yaw")
            node.send_goal(i, goal, args.timeout)
            print(f"Goal {i} reached: {node.events[-1]['final_error_m']:.3f} m error")
        node.goal_index = len(goals)
        node.spin_for(args.hold_after)
        ok = True
        print("Performance suite completed")
    except (RuntimeError, TimeoutError) as exc:
        print(f"TEST ABORTED: {exc}")
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
