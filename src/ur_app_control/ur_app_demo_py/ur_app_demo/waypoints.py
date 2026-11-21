#!/usr/bin/env python3
"""
Move a UR robot through a list of absolute waypoints using pymoveit2.

Same setup as simple_move.py, but instead of one target it visits every
entry in WAYPOINTS, top to bottom. Requires a running MoveIt 2 move_group.

If stop_on_force stops the robot (/force_stop == True), the script waits until
/force_stop goes False, then re-plans to the SAME waypoint and continues.

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

from scipy.spatial.transform import Rotation
from pymoveit2 import MoveIt2

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool

# Reuse the robot configuration from simple_move.py
from ur_app_demo.simple_move import GROUP, JOINT_NAMES, BASE_LINK, END_EFFECTOR


# ---------------------------------------------------------------------------
# EDIT HERE
# ---------------------------------------------------------------------------
""""
WAYPOINTS = [
    {"name": "home",    "joints": [1, -119, -115, -37, 90, 90]},
    {"name": "point_A",  "joints": [1, -141, -74, -54, 90, 90]},
    {"name": "point_B",  "joints": [-16, -156, -40, -71, 90, 90]},
    {"name": "point_C",  "joints": [-41, -120, -110, -40, 90, 90]},
    {"name": "point_D",  "joints": [-4, -97, -79, -90, 90, 90]},
    {"name": "home",    "joints": [1, -119, -115, -37, 90, 90]},
]
"""
WAYPOINTS = [
    #{"name": "home",    "joints": [0, -90, 90, -90, -90, 0]},
    {"name": "point_A", "pose": [0.40,  0.20, 0.43, 180, 0, 0], "linear": False},
    {"name": "point_B", "pose": [0.40, -0.20, 0.40, 180, 0, 0], "linear": False},
    {"name": "point_C", "pose": [0.30,  0.10, 0.25, 180, 0, 0], "linear": False},
    #{"name": "home",    "joints": [0, -90, 90, -90, -90, 0]},
]

SPEED = 0.2          # 0.0-1.0, fraction of max velocity
ACCELERATION = 0.2   # 0.0-1.0, fraction of max acceleration
PAUSE_SECONDS = 0.5  # pause at each waypoint
RESUME_DELAY = 1.0   # s to wait after /force_stop goes False before moving again
# ---------------------------------------------------------------------------


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

        self._force_stop = False   # current state from stop_on_force
        self._stop_seen = False    # a stop happened during the current move
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Bool, "force_stop", self._on_force_stop, latched)

    def _on_force_stop(self, msg):
        self._force_stop = msg.data
        if msg.data:
            self._stop_seen = True

    def _wait_release(self):
        """Block while the force stop is active, then wait RESUME_DELAY."""
        if not self._force_stop:
            return
        self.get_logger().warn("Force stop active, waiting for release")
        while self._force_stop and rclpy.ok():
            time.sleep(0.05)
        time.sleep(RESUME_DELAY)
        self.get_logger().info("Released, resuming")

    def move_to_waypoint(self, waypoint):
        """Plan and execute one waypoint. Returns True on success."""

        if "joints" in waypoint:
            joints = [math.radians(a) for a in waypoint["joints"]]
            self.moveit2.move_to_configuration(joints)
        elif "pose" in waypoint:
            x, y, z, roll, pitch, yaw = waypoint["pose"]
            quat = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=True).as_quat()
            self.moveit2.move_to_pose(
                position=[x, y, z],
                quat_xyzw=quat,
                frame_id=BASE_LINK,
                target_link=END_EFFECTOR,
                cartesian=waypoint.get("linear", False),
            )
        else:
            raise SyntaxError("The move has to be either joints or pose")

        return self.moveit2.wait_until_executed()

    def run(self):
        for i, waypoint in enumerate(WAYPOINTS, start=1):
            name = waypoint.get("name", f"#{i}")
            self.get_logger().info(f"[{i}/{len(WAYPOINTS)}] Moving to {name}")

            while rclpy.ok():
                self._wait_release()           # also covers a stop during the pause
                self._stop_seen = False
                if self.move_to_waypoint(waypoint):
                    break
                if not self._stop_seen:        # failed for another reason
                    self.get_logger().error(f"Failed to reach {name}, stopping")
                    return
                # stopped by force sensor: loop, wait for release, re-plan to same target

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