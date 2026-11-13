#!/usr/bin/env python3
"""
Move a UR robot through a list of absolute waypoints using pymoveit2.

Same setup as simple_move.py, but instead of one target it visits every
entry in WAYPOINTS, top to bottom. Requires a running MoveIt 2 move_group.

Run:
    ros2 run ur_app_demo waypoints

Each waypoint has a "name" and EITHER a "pose" OR "joints":

    "pose":   [x, y, z, roll, pitch, yaw]   tool0 in base_link
              x, y, z in metres, roll/pitch/yaw in degrees
              add "linear": True for a straight-line move
    "joints": [base, shoulder, elbow, wrist1, wrist2, wrist3]
              all in degrees
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from pymoveit2 import MoveIt2

# Reuse the robot configuration from simple_move.py
from ur_app_demo.simple_move import GROUP, JOINT_NAMES, BASE_LINK, END_EFFECTOR


# ---------------------------------------------------------------------------
# EDIT HERE
# ---------------------------------------------------------------------------
WAYPOINTS = [
    {"name": "home",    "joints": [1, -119, -115, -37, 90, 90]},
    {"name": "point_A",  "joints": [1, -141, -74, -54, 90, 90]},
    {"name": "point_B",  "joints": [-16, -156, -40, -71, 90, 90]},
    {"name": "point_C",  "joints": [-41, -120, -110, -40, 90, 90]},
    {"name": "point_D",  "joints": [-4, -97, -79, -90, 90, 90]},
    {"name": "home",    "joints": [1, -119, -115, -37, 90, 90]},
]

SPEED = 0.2          # 0.0-1.0, fraction of max velocity
ACCELERATION = 0.2   # 0.0-1.0, fraction of max acceleration
PAUSE_SECONDS = 0.5  # pause at each waypoint
# ---------------------------------------------------------------------------


def rpy_to_quat(roll, pitch, yaw):
    """Roll, pitch, yaw in degrees -> quaternion [x, y, z, w]."""
    r, p, y = (math.radians(a) / 2.0 for a in (roll, pitch, yaw))
    cr, sr = math.cos(r), math.sin(r)
    cp, sp = math.cos(p), math.sin(p)
    cy, sy = math.cos(y), math.sin(y)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


class WaypointMove(Node):

    def __init__(self):
        super().__init__("waypoint_move")

        self.moveit2 = MoveIt2(
            node=self,
            joint_names=JOINT_NAMES,
            base_link_name=BASE_LINK,
            end_effector_name=END_EFFECTOR,
            group_name=GROUP,
        )
        self.moveit2.max_velocity = SPEED
        self.moveit2.max_acceleration = ACCELERATION

        self.get_logger().info("pymoveit2 initialized")

    def move_to_waypoint(self, waypoint):
        """Plan and execute one waypoint. Returns True on success."""

        if "joints" in waypoint:
            joints = [math.radians(a) for a in waypoint["joints"]]
            self.moveit2.move_to_configuration(joints)
        else:
            x, y, z, roll, pitch, yaw = waypoint["pose"]
            self.moveit2.move_to_pose(
                position=[x, y, z],
                quat_xyzw=rpy_to_quat(roll, pitch, yaw),
                frame_id=BASE_LINK,
                target_link=END_EFFECTOR,
                cartesian=waypoint.get("linear", False),
            )

        return self.moveit2.wait_until_executed()

    def run(self):
        for i, waypoint in enumerate(WAYPOINTS, start=1):
            name = waypoint.get("name", f"#{i}")
            self.get_logger().info(f"[{i}/{len(WAYPOINTS)}] Moving to {name}")

            if not self.move_to_waypoint(waypoint):
                self.get_logger().error(f"Failed to reach {name}, stopping")
                return

            time.sleep(PAUSE_SECONDS)

        self.get_logger().info("All waypoints completed")


def main():

    rclpy.init()

    node = WaypointMove()

    # Run ROS callbacks in a background thread
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    executor_thread = threading.Thread(
        target=executor.spin,
        daemon=True,
    )
    executor_thread.start()

    try:
        node.run()

    except KeyboardInterrupt:
        node.get_logger().info("Interrupted")

    finally:
        executor.shutdown()
        executor_thread.join()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
