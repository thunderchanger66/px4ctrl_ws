#!/usr/bin/env python3
"""Send one EGO goal after the planner and controller interfaces are ready."""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Empty, Float64, String


def parse_args():
    parser = argparse.ArgumentParser(description="Send a goal in the EGO planning frame")
    parser.add_argument("x", type=float)
    parser.add_argument("y", type=float)
    parser.add_argument("z", type=float)
    parser.add_argument("yaw_deg", nargs="?", type=float)
    parser.add_argument("--auto-yaw", action="store_true")
    parser.add_argument("--frame", default="lidar")
    parser.add_argument("--goal-topic", default="/goal_pose")
    parser.add_argument("--manual-yaw-topic", default="/px4ctrl/manual_yaw")
    parser.add_argument("--clear-manual-yaw-topic", default="/px4ctrl/clear_manual_yaw")
    parser.add_argument("--wait-timeout", type=float, default=30.0)
    parser.add_argument("--publish-count", type=int, default=5)
    args = parser.parse_args()
    if args.auto_yaw == (args.yaw_deg is not None):
        parser.error("provide yaw_deg, or use --auto-yaw")
    values = [args.x, args.y, args.z]
    if args.yaw_deg is not None:
        values.append(args.yaw_deg)
    if not all(math.isfinite(value) for value in values):
        parser.error("coordinates and yaw must be finite")
    if args.wait_timeout <= 0.0 or args.publish_count < 1:
        parser.error("wait-timeout must be positive and publish-count must be at least one")
    return args


class GoalSender(Node):
    def __init__(self, args):
        super().__init__("px4ctrl_manual_goal")
        self.args = args
        self.goal_pub = self.create_publisher(PoseStamped, args.goal_topic, 10)
        self.yaw_pub = self.create_publisher(Float64, args.manual_yaw_topic, 10)
        self.clear_yaw_pub = self.create_publisher(Empty, args.clear_manual_yaw_topic, 10)
        self.status = []
        self.create_subscription(String, "/px4ctrl/status", self.status.append, 10)

    def wait_for_goal_subscriber(self):
        deadline = time.monotonic() + self.args.wait_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self.goal_pub.get_subscription_count() > 0:
                return
            rclpy.spin_once(self, timeout_sec=0.1)
        raise RuntimeError(f"no subscriber discovered for {self.args.goal_topic}")

    def send(self):
        if self.args.auto_yaw:
            self.clear_yaw_pub.publish(Empty())
            heading = "EGO path heading"
        else:
            yaw = Float64()
            yaw.data = math.radians(self.args.yaw_deg)
            self.yaw_pub.publish(yaw)
            heading = f"{self.args.yaw_deg:g} deg"

        goal = PoseStamped()
        goal.header.frame_id = self.args.frame
        goal.pose.position.x = self.args.x
        goal.pose.position.y = self.args.y
        goal.pose.position.z = self.args.z
        yaw_rad = 0.0 if self.args.auto_yaw else math.radians(self.args.yaw_deg)
        goal.pose.orientation.z = math.sin(yaw_rad / 2.0)
        goal.pose.orientation.w = math.cos(yaw_rad / 2.0)
        for _ in range(self.args.publish_count):
            goal.header.stamp = self.get_clock().now().to_msg()
            self.goal_pub.publish(goal)
            rclpy.spin_once(self, timeout_sec=0.1)
        print(
            f"Sent goal ({self.args.x:g}, {self.args.y:g}, {self.args.z:g}) m "
            f"in {self.args.frame}; heading: {heading}."
        )


def main():
    args = parse_args()
    rclpy.init()
    node = GoalSender(args)
    try:
        node.wait_for_goal_subscriber()
        node.send()
    except (RuntimeError, KeyboardInterrupt) as exc:
        if str(exc):
            print(f"manual_goal: {exc}", file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
