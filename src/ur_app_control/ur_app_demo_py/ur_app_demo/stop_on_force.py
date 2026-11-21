#!/usr/bin/env python3
"""Stop MoveIt trajectory execution when /arduino_force crosses a threshold.

Also publishes the stop state on /force_stop (std_msgs/Bool, latched):
  True  -> threshold hit, stop sent
  False -> value back past the threshold by `hysteresis`, safe to resume
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32, String


class StopOnForce(Node):
    def __init__(self):
        super().__init__('stop_on_force')
        self.declare_parameter('threshold', 512)
        self.declare_parameter('trigger_above', True)
        self.declare_parameter('hysteresis', 20)  # counts past threshold to release
        self._tripped = False
        self._pub = self.create_publisher(
            String, '/trajectory_execution_event', 10)

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._state_pub = self.create_publisher(Bool, '/force_stop', latched)
        self._state_pub.publish(Bool(data=False))

        self.create_subscription(Int32, '/arduino_force', self._on_value, 10)

    def _on_value(self, msg):
        thr = self.get_parameter('threshold').value
        above = self.get_parameter('trigger_above').value
        hyst = self.get_parameter('hysteresis').value
        if above:
            hit, clear = msg.data >= thr, msg.data < thr - hyst
        else:
            hit, clear = msg.data <= thr, msg.data > thr + hyst

        if hit and not self._tripped:       # fire once per crossing
            self._tripped = True
            # state first, so the motion script knows why the abort happens
            self._state_pub.publish(Bool(data=True))
            self._pub.publish(String(data='stop'))
            self.get_logger().warn(f'threshold hit ({msg.data}), stop sent')
        elif clear and self._tripped:
            self._tripped = False           # re-arm and release
            self._state_pub.publish(Bool(data=False))
            self.get_logger().info(f'below threshold ({msg.data}), released')


def main(args=None):
    rclpy.init(args=args)
    node = StopOnForce()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()