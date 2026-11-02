#!/usr/bin/env python3
"""Record EGO position commands and PX4 local position for a bounded trial."""

import argparse
import csv
import math
import time

import rclpy
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus
from quadrotor_msgs.msg import PositionCommand
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class TrackEval(Node):
    def __init__(self, args):
        super().__init__("px4ctrl_track_eval")
        self.args = args
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.rows = []
        self.command = None
        self.position = None
        self.status = None
        self.started = time.monotonic()
        self.create_subscription(
            PositionCommand, "/drone_0_planning/pos_cmd", self.command_cb, 20
        )
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.position_cb,
            sensor_qos,
        )
        self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status_v4", self.status_cb, sensor_qos
        )
        self.timer = self.create_timer(0.05, self.record)

    def command_cb(self, msg):
        if msg.trajectory_flag != PositionCommand.TRAJECTORY_STATUS_READY:
            return
        self.command = msg

    def position_cb(self, msg):
        self.position = msg

    def status_cb(self, msg):
        self.status = msg

    def record(self):
        now = time.monotonic()
        if now - self.started >= self.args.duration:
            self.write_csv()
            rclpy.shutdown()
            return
        c = self.command
        p = self.position
        s = self.status
        if c is None or p is None:
            return
        self.rows.append({
            "elapsed_s": now - self.started,
            "cmd_x_lidar": c.position.x,
            "cmd_y_lidar": c.position.y,
            "cmd_z_lidar": c.position.z,
            "cmd_vx_lidar": c.velocity.x,
            "cmd_vy_lidar": c.velocity.y,
            "cmd_vz_lidar": c.velocity.z,
            "cmd_yaw_lidar": c.yaw,
            "px4_x_ned": p.x,
            "px4_y_ned": p.y,
            "px4_z_ned": p.z,
            "px4_vx_ned": p.vx,
            "px4_vy_ned": p.vy,
            "px4_vz_ned": p.vz,
            "xy_valid": int(p.xy_valid),
            "z_valid": int(p.z_valid),
            "dead_reckoning": int(p.dead_reckoning),
            "nav_state": s.nav_state if s else -1,
            "arming_state": s.arming_state if s else -1,
        })

    def write_csv(self):
        fields = [
            "elapsed_s", "cmd_x_lidar", "cmd_y_lidar", "cmd_z_lidar",
            "cmd_vx_lidar", "cmd_vy_lidar", "cmd_vz_lidar", "cmd_yaw_lidar",
            "px4_x_ned", "px4_y_ned", "px4_z_ned", "px4_vx_ned",
            "px4_vy_ned", "px4_vz_ned", "xy_valid", "z_valid",
            "dead_reckoning", "nav_state", "arming_state",
        ]
        with open(self.args.csv, "w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(self.rows)
        self.get_logger().info(f"saved {len(self.rows)} samples to {self.args.csv}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--csv", required=True)
    parser.add_argument("--duration", type=float, default=60.0)
    args = parser.parse_args()
    if not math.isfinite(args.duration) or args.duration <= 0.0:
        parser.error("duration must be positive")
    rclpy.init()
    node = TrackEval(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.write_csv()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
