#!/usr/bin/env python3
"""Read integer lines from an Arduino over serial and publish as Int32."""
import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import Int32


class ArduinoBridge(Node):
    def __init__(self):
        super().__init__('arduino_bridge')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self._ser = serial.Serial(
            self.get_parameter('port').value,
            self.get_parameter('baud').value,
            timeout=0)
        self._buf = b''
        self._pub = self.create_publisher(Int32, 'arduino_analog', 10)
        self.create_timer(0.01, self._poll)

    def _poll(self):
        self._buf += self._ser.read(self._ser.in_waiting)
        while b'\n' in self._buf:
            line, self._buf = self._buf.split(b'\n', 1)
            try:
                self._pub.publish(Int32(data=int(line.strip())))
            except ValueError:
                self.get_logger().warn(f'bad line: {line!r}')


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
