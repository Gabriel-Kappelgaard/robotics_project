#!/usr/bin/env python3
"""
Move the UR robot through a list of waypoints, one after another.

HOW TO USE
    1. Edit the WAYPOINTS list below.
    2. Rebuild:  colcon build --packages-select ur_app_bringup && source install/setup.bash
    3. Run:      ros2 run ur_app_bringup waypoints

Each waypoint is a dict and must have EITHER "pose" OR "joints":

    Pose waypoint (absolute position of the tool, relative to base_link):
        {"name": "above_table", "pose": [x, y, z, roll, pitch, yaw]}
            x, y, z           in metres
            roll, pitch, yaw  in degrees
        Optional: "linear": True  -> move in a straight line (Cartesian path)

    Joint waypoint (absolute joint angles):
        {"name": "home", "joints": [base, shoulder, elbow, wrist1, wrist2, wrist3]}
            all six angles in degrees

Uses pymoveit2, so the UR driver and MoveIt must already be running
(see README.md).
"""

import math
import time
from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from pymoveit2 import MoveIt2


# =============================================================================
# EDIT HERE: waypoints, visited in order from top to bottom
# =============================================================================
WAYPOINTS = [
    {"name": "home",        "joints": [0, -90, 90, -90, -90, 0]},
    {"name": "point_A",     "pose": [0.40,  0.20, 0.40, 180, 0, 0]},
    {"name": "point_B",     "pose": [0.40, -0.20, 0.40, 180, 0, 0], "linear": True},
    {"name": "point_C",     "pose": [0.30,  0.00, 0.25, 180, 0, 0]},
    {"name": "home",        "joints": [0, -90, 90, -90, -90, 0]},
]

# Settings
SPEED = 0.2          # 0.0-1.0, fraction of max joint velocity
ACCELERATION = 0.2   # 0.0-1.0, fraction of max joint acceleration
PAUSE_SECONDS = 0.5  # pause at each waypoint
REPEAT = 1           # how many times to run the whole list (0 = forever)
STOP_ON_FAILURE = True
# =============================================================================


# Robot configuration (UR + ur_moveit_config)
GROUP = "ur_manipulator"
BASE_LINK = "base_link"
END_EFFECTOR = "tool0"
JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]


def rpy_deg_to_quat_xyzw(roll, pitch, yaw):
    """Roll/pitch/yaw in degrees (fixed XYZ axes) -> quaternion [x, y, z, w]."""
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


def check_waypoints(waypoints):
    """Catch typos before the robot moves."""
    for i, wp in enumerate(waypoints):
        label = f"Waypoint {i} ({wp.get('name', '?')})"
        has_pose, has_joints = "pose" in wp, "joints" in wp
        if has_pose == has_joints:
            raise ValueError(f"{label}: needs exactly one of 'pose' or 'joints'")
        if has_pose and len(wp["pose"]) != 6:
            raise ValueError(f"{label}: 'pose' must be [x, y, z, roll, pitch, yaw]")
        if has_joints and len(wp["joints"]) != 6:
            raise ValueError(f"{label}: 'joints' must have 6 angles")


class WaypointRunner(Node):
    def __init__(self):
        super().__init__("waypoint_runner")
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=JOINT_NAMES,
            base_link_name=BASE_LINK,
            end_effector_name=END_EFFECTOR,
            group_name=GROUP,
            callback_group=ReentrantCallbackGroup(),
        )
        self.moveit2.max_velocity = SPEED
        self.moveit2.max_acceleration = ACCELERATION

    def go_to(self, wp):
        """Send one waypoint to MoveIt and wait until it is executed."""
        if "joints" in wp:
            self.moveit2.move_to_configuration([math.radians(a) for a in wp["joints"]])
        else:
            x, y, z, roll, pitch, yaw = wp["pose"]
            self.moveit2.move_to_pose(
                position=[x, y, z],
                quat_xyzw=rpy_deg_to_quat_xyzw(roll, pitch, yaw),
                frame_id=BASE_LINK,
                target_link=END_EFFECTOR,
                cartesian=wp.get("linear", False),
            )
        result = self.moveit2.wait_until_executed()
        # Older pymoveit2 versions return None instead of True/False.
        return result is None or bool(result)

    def run(self, waypoints):
        lap = 0
        while REPEAT == 0 or lap < REPEAT:
            lap += 1
            for i, wp in enumerate(waypoints):
                name = wp.get("name", f"#{i}")
                self.get_logger().info(f"[{i + 1}/{len(waypoints)}] -> {name}")
                if not self.go_to(wp):
                    self.get_logger().error(f"Failed to reach '{name}'")
                    if STOP_ON_FAILURE:
                        return False
                time.sleep(PAUSE_SECONDS)
        self.get_logger().info("All waypoints done.")
        return True


def main():
    check_waypoints(WAYPOINTS)
    rclpy.init()
    node = WaypointRunner()

    executor = MultiThreadedExecutor(2)
    executor.add_node(node)
    Thread(target=executor.spin, daemon=True).start()
    node.create_rate(1.0).sleep()  # let joint states arrive

    try:
        node.run(WAYPOINTS)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
