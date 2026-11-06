"""moveJ / moveL-style helpers on top of MoveIt's Pilz planner (PTP / LIN).

Pilz plans, MoveIt executes through the driver's trajectory controller, so the
same code runs on mock hardware and on a real UR5e.

Conventions (as in URScript):
  - joints: 6 values in radians
  - pose:   [x, y, z, rx, ry, rz], metres + rotation vector, in base_link
Speed and acceleration are scaling factors (0..1] of the configured limits,
not absolute m/s or rad/s.
"""
import math
import time

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume, Constraints, JointConstraint, MoveItErrorCodes,
    OrientationConstraint, PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformException, TransformListener

GROUP = 'ur_manipulator'
PIPELINE = 'pilz_industrial_motion_planner'
ARM_JOINTS = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]


def rotvec_to_quat(rx, ry, rz):
    angle = math.sqrt(rx * rx + ry * ry + rz * rz)
    if angle < 1e-9:
        return 0.0, 0.0, 0.0, 1.0
    s = math.sin(angle / 2.0) / angle
    return rx * s, ry * s, rz * s, math.cos(angle / 2.0)


def quat_to_rotvec(x, y, z, w):
    if w < 0.0:
        x, y, z, w = -x, -y, -z, -w
    s = math.sqrt(x * x + y * y + z * z)
    if s < 1e-9:
        return 0.0, 0.0, 0.0
    k = 2.0 * math.atan2(s, w) / s
    return x * k, y * k, z * k


class UrMotion(Node):
    def __init__(self, base_frame='base_link', tcp_link='tool0'):
        super().__init__('ur_motion')
        self._base = base_frame
        self._tcp = tcp_link
        self._q = {}
        self.create_subscription(JointState, 'joint_states', self._on_js, 10)
        self._client = ActionClient(self, MoveGroup, 'move_action')
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    def _on_js(self, msg):
        self._q.update(zip(msg.name, msg.position))

    def wait(self, seconds):
        """Spin the node (keeps joint_states / TF fresh) for `seconds`."""
        t_end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < t_end:
            rclpy.spin_once(self, timeout_sec=0.02)

    # ---- state ---------------------------------------------------------
    def current_joints(self, timeout=5.0):
        self.wait(0.2)
        t_end = time.monotonic() + timeout
        while not all(j in self._q for j in ARM_JOINTS):
            if time.monotonic() > t_end:
                raise RuntimeError('no joint_states received')
            self.wait(0.1)
        return [self._q[j] for j in ARM_JOINTS]

    def current_pose(self, timeout=5.0):
        t_end = time.monotonic() + timeout
        while True:
            self.wait(0.2)
            try:
                t = self._tf_buffer.lookup_transform(
                    self._base, self._tcp, rclpy.time.Time())
                break
            except TransformException:
                if time.monotonic() > t_end:
                    raise RuntimeError(
                        f'TF {self._base} -> {self._tcp} not available')
        p, q = t.transform.translation, t.transform.rotation
        return [p.x, p.y, p.z, *quat_to_rotvec(q.x, q.y, q.z, q.w)]

    # ---- motions -------------------------------------------------------
    def move_j(self, q, vel_scale=0.1, acc_scale=0.1):
        """Joint-space point-to-point move (Pilz PTP). Returns True on success."""
        c = Constraints()
        for name, pos in zip(ARM_JOINTS, q):
            c.joint_constraints.append(JointConstraint(
                joint_name=name, position=float(pos),
                tolerance_above=1e-3, tolerance_below=1e-3, weight=1.0))
        return self._send('PTP', c, vel_scale, acc_scale)

    def move_l(self, pose, vel_scale=0.1, acc_scale=0.1):
        """Straight-line TCP move (Pilz LIN) to [x, y, z, rx, ry, rz]."""
        x, y, z, rx, ry, rz = pose
        qx, qy, qz, qw = rotvec_to_quat(rx, ry, rz)

        centre = Pose()
        centre.position.x, centre.position.y, centre.position.z = x, y, z
        centre.orientation.w = 1.0
        pc = PositionConstraint()
        pc.header.frame_id = self._base
        pc.link_name = self._tcp
        pc.constraint_region = BoundingVolume(
            primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE,
                                       dimensions=[1e-3])],
            primitive_poses=[centre])
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header.frame_id = self._base
        oc.link_name = self._tcp
        oc.orientation.x, oc.orientation.y = qx, qy
        oc.orientation.z, oc.orientation.w = qz, qw
        oc.absolute_x_axis_tolerance = 1e-3
        oc.absolute_y_axis_tolerance = 1e-3
        oc.absolute_z_axis_tolerance = 1e-3
        oc.weight = 1.0

        c = Constraints(position_constraints=[pc], orientation_constraints=[oc])
        return self._send('LIN', c, vel_scale, acc_scale)

    # ---- internals -----------------------------------------------------
    def _send(self, planner_id, constraints, vel_scale, acc_scale):
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('move_action server not available')
            return False
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = GROUP
        req.pipeline_id = PIPELINE
        req.planner_id = planner_id
        req.num_planning_attempts = 1
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = float(vel_scale)
        req.max_acceleration_scaling_factor = float(acc_scale)
        req.goal_constraints.append(constraints)
        goal.planning_options.plan_only = False

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if not handle.accepted:
            self.get_logger().error(f'{planner_id} goal rejected')
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        code = result_future.result().result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            self.get_logger().error(
                f'{planner_id} failed, MoveIt error code {code}')
            return False
        return True
