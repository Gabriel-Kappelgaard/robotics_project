#!/usr/bin/env python3
"""Minimal MoveIt client: nudge shoulder_pan_joint by `delta` rad from the
current pose, plan with MoveIt, execute. Same node for mock and real UR5e."""
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
from sensor_msgs.msg import JointState

ARM_JOINTS = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]


class MoveToJoints(Node):
    def __init__(self):
        super().__init__('move_to_joints')
        self.declare_parameter('delta', 0.2)           # rad, added to shoulder_pan
        self.declare_parameter('velocity_scale', 0.1)  # keep low on real hardware
        self.declare_parameter('accel_scale', 0.1)
        self._latest = {}
        self.create_subscription(JointState, 'joint_states', self._on_js, 10)
        self._client = ActionClient(self, MoveGroup, 'move_action')

    def _on_js(self, msg):
        self._latest = dict(zip(msg.name, msg.position))

    def _have_state(self):
        return all(j in self._latest for j in ARM_JOINTS)

    def _wait_for_state(self, timeout=5.0):
        t_end = time.monotonic() + timeout
        while rclpy.ok() and not self._have_state() and time.monotonic() < t_end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self._have_state()

    def run(self):
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('move_action server not available')
            return 1
        if not self._wait_for_state():
            self.get_logger().error('no joint_states received')
            return 1

        target = [self._latest[j] for j in ARM_JOINTS]
        target[0] += self.get_parameter('delta').value

        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = 'ur_manipulator'
        req.num_planning_attempts = 5
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = self.get_parameter('velocity_scale').value
        req.max_acceleration_scaling_factor = self.get_parameter('accel_scale').value

        constraints = Constraints()
        for name, pos in zip(ARM_JOINTS, target):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        req.goal_constraints.append(constraints)
        goal.planning_options.plan_only = False

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if not handle.accepted:
            self.get_logger().error('goal rejected')
            return 1

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        code = result_future.result().result.error_code.val
        if code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info('motion succeeded')
            return 0
        self.get_logger().error(f'motion failed, MoveIt error code {code}')
        return 1


def main(args=None):
    rclpy.init(args=args)
    node = MoveToJoints()
    try:
        exit_code = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
