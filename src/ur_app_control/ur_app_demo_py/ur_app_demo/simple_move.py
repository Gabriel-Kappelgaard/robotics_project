
#!/usr/bin/env python3
"""
Move a UR robot to a Cartesian pose using pymoveit2.

Requires a running MoveIt 2 move_group.
"""

import threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from pymoveit2 import MoveIt2


# UR robot configuration
GROUP = "ur_manipulator"

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

BASE_LINK = "base_link"
END_EFFECTOR = "tool0"


class SimpleMove(Node):

    def __init__(self):
        super().__init__("simple_move")

        # Initialize pymoveit2
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=JOINT_NAMES,
            base_link_name=BASE_LINK,
            end_effector_name=END_EFFECTOR,
            group_name=GROUP,
        )

        self.get_logger().info("pymoveit2 initialized")

    def move_to_target(self):

        # Target position (metres)
        position = [0.4, 0.2, 0.4]

        # Quaternion (x, y, z, w)
        orientation = [1.0, 0.0, 0.0, 0.0]

        self.get_logger().info("Planning and executing pose goal...")

        self.moveit2.move_to_pose(
            position=position,
            quat_xyzw=orientation,
            frame_id=BASE_LINK,
            target_link=END_EFFECTOR,
        )

        # Wait until execution finishes
        success = self.moveit2.wait_until_executed()

        if success:
            self.get_logger().info("Motion completed successfully")
        else:
            self.get_logger().error("Motion execution failed")


def main():

    rclpy.init()

    node = SimpleMove()

    # Run ROS callbacks in a background thread
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    executor_thread = threading.Thread(
        target=executor.spin,
        daemon=True,
    )
    executor_thread.start()

    try:
        node.move_to_target()

    except KeyboardInterrupt:
        node.get_logger().info("Interrupted")

    finally:
        executor.shutdown()
        executor_thread.join()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
