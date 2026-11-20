#!/usr/bin/env python3
"""Stop MoveIt trajectory execution when /arduino_analog crosses a threshold."""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String


class StopOnForce(Node):
    def __init__(self):
        super().__init__('stop_on_force')
        self.declare_parameter('threshold', 512)
        self.declare_parameter('trigger_above', True)
        self._tripped = False
        self._pub = self.create_publisher(
            String, '/trajectory_execution_event', 10)
        self.create_subscription(Int32, '/arduino_force', self._on_value, 10)

    def _on_value(self, msg):
        thr = self.get_parameter('threshold').value
        above = self.get_parameter('trigger_above').value
        hit = msg.data >= thr if above else msg.data <= thr
        if hit and not self._tripped:       # fire once per crossing
            self._tripped = True
            self._pub.publish(String(data='stop'))
            self.get_logger().warn(f'threshold hit ({msg.data}), stop sent')
        elif not hit:
            self._tripped = False           # re-arm


def main(args=None):
    rclpy.init(args=args)
    node = StopOnForce()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()