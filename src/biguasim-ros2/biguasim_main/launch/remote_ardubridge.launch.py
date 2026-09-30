"""Bring up the ArduPilot bridge against a world running somewhere else.

The local `ardubridge.launch.py` starts a node that owns the simulator. This
one starts a node that only owns a connection to it, so the same world can hold
several vehicles from several machines at once.

The scenario YAML is still read locally -- it decides which agent to spawn, its
sensors and their rates -- but `package_name`/`world` in it must match what the
world process is actually running, or the build check refuses the connection.

    ros2 launch biguasim_main remote_ardubridge.launch.py \
        world_address:=fakenatty.tail678f03.ts.net world_port:=8770 \
        agent_name:=HolybroX500
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    config_dir = str(Path(get_package_share_directory('biguasim_main')) / 'config')
    # A substitution list, not an f-string: agent_name resolves when the node
    # starts, not while this description is being built.
    params_file = [config_dir, '/config-', LaunchConfiguration('agent_name'), '.yaml']

    args = [
        DeclareLaunchArgument('agent_name', default_value='HolybroX500',
                              description='Which airframe to spawn: reads '
                                          'config/config-<agent_name>.yaml. Its '
                                          'package_name/world must match the '
                                          'world process, or the build check '
                                          'refuses the connection.'),
        DeclareLaunchArgument('world_address', default_value='127.0.0.1',
                              description='Host running the world process.'),
        DeclareLaunchArgument('world_port', default_value='8770',
                              description='Its request port; state is port+1.'),
        DeclareLaunchArgument('instance', default_value='0',
                              description='ArduPilot SITL instance. Shifts every '
                                          'port by 10x this, which is how a second '
                                          'vehicle avoids the first.'),
        DeclareLaunchArgument('report_every', default_value='0',
                              description='Log a frame/skip count every N ticks. 0 is silent.'),
        DeclareLaunchArgument('phase', default_value='1',
                              description='Competition phase. 3 and 4 fly the '
                                          'Kopis, whose Mid-360 needs a spinning '
                                          'sensor -- which this path CANNOT do, '
                                          'the world owning it. Given here only '
                                          'so the node can say so.'),
    ]

    node = launch_ros.actions.Node(
        name='remote_ardubridge_node',
        package='biguasim_main',
        executable='remote_ardubridge_node',
        namespace='biguasim',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'params_file': ParameterValue(params_file, value_type=str),
            'world_address': LaunchConfiguration('world_address'),
            'world_port': ParameterValue(LaunchConfiguration('world_port'), value_type=int),
            'instance': ParameterValue(LaunchConfiguration('instance'), value_type=int),
            'report_every': ParameterValue(LaunchConfiguration('report_every'), value_type=int),
            'phase': ParameterValue(LaunchConfiguration('phase'), value_type=int),
        }],
    )

    return LaunchDescription(args + [node])
