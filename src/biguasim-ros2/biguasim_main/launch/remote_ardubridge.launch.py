"""Bring up the ArduPilot bridge against a world running somewhere else.

The local `ardubridge.launch.py` starts a node that owns the simulator. This
one starts a node that only owns a connection to it, so the same world can hold
several vehicles from several machines at once.

The scenario YAML is still read locally -- it decides which agent to spawn, its
sensors and their rates -- but `package_name`/`world` in it must match what the
world process is actually running, or the build check refuses the connection.

    ros2 launch biguasim_main remote_ardubridge.launch.py \
        world_address:=fakenatty.tail678f03.ts.net world_port:=8770
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, ParameterValue
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    base = Path(get_package_share_directory('biguasim_main'))
    params_file = base / 'config' / 'config.yaml'

    args = [
        DeclareLaunchArgument('world_address', default_value='127.0.0.1',
                              description='Host running the world process.'),
        DeclareLaunchArgument('world_port', default_value='8770',
                              description='Its request port; state is port+1.'),
        DeclareLaunchArgument('report_every', default_value='0',
                              description='Log a frame/skip count every N ticks. 0 is silent.'),
    ]

    node = launch_ros.actions.Node(
        name='remote_ardubridge_node',
        package='biguasim_main',
        executable='remote_ardubridge_node',
        namespace='biguasim',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'params_file': str(params_file),
            'world_address': LaunchConfiguration('world_address'),
            'world_port': ParameterValue(LaunchConfiguration('world_port'), value_type=int),
            'report_every': ParameterValue(LaunchConfiguration('report_every'), value_type=int),
        }],
    )

    return LaunchDescription(args + [node])
