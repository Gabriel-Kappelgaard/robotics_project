from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baud', default_value='115200'),
        Node(
            package='ur_app_demo',  # must match <name> in package.xml
            executable='arduino_bridge',
            name='arduino_bridge',
            output='screen',
            respawn=True,       # restarts if the serial port drops
            respawn_delay=2.0,
            parameters=[{
                'port': LaunchConfiguration('port'),
                'baud': LaunchConfiguration('baud'),
            }],
        ),
        Node(
            package='ur_app_demo',
            executable='stop_on_force',
            parameters=[{
                'threshold': 512
            }],
        ),
    ])