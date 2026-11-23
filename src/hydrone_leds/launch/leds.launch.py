"""The LED node on its own.

    ros2 launch hydrone_leds leds.launch.py                  # Jetson, white
    ros2 launch hydrone_leds leds.launch.py backend:=fake    # no hardware
    ros2 launch hydrone_leds leds.launch.py color:=red
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('color', default_value='white'),
        DeclareLaunchArgument('backend', default_value='sysfs',
                              description='sysfs (Jetson GPIO) or fake'),
        Node(package='hydrone_leds', executable='led_node', output='screen',
             parameters=[{'color': LaunchConfiguration('color'),
                          'backend': LaunchConfiguration('backend')}]),
    ])
