import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading

from pymoveit2 import MoveIt2

from scipy.spatial.transform import Rotation


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



class Move(Node):
    def __init__(self):
        super().__init__("simple_move") #TODO figure out what this means

        # Initialize pymoveit2
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=JOINT_NAMES,
            base_link_name=BASE_LINK,
            end_effector_name=END_EFFECTOR,
            group_name=GROUP,
        )

        self.get_logger().info("pymoveit2 initialized")

    def move_to_pos(self, pos):
        """Plan and execute one waypoint. Returns True on success."""

        if "joints" in pos:
            joints = [pos.radians(a) for a in pos["joints"]]
            self.moveit2.move_to_configuration(joints)
        elif "pose" in pos:
            x, y, z, roll, pitch, yaw = pos["pose"]
            quat = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=True).as_quat()
            self.moveit2.move_to_pose(
                position=[x, y, z],
                quat_xyzw=quat,
                frame_id=BASE_LINK,
                target_link=END_EFFECTOR,
                cartesian=pos.get("linear", False),
            )
        else:
            raise SyntaxError("The move has to be either joints or pose")

        return self.moveit2.wait_until_executed()




def main():

    rclpy.init()

    node = Move()

    # Run ROS callbacks in a background thread
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    executor_thread = threading.Thread(
        target=executor.spin,
        daemon=True,
    )
    executor_thread.start()

    try:
        node.move_to_pos()

    except KeyboardInterrupt:
        node.get_logger().info("Interrupted")

    finally:
        executor.shutdown()
        executor_thread.join()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()