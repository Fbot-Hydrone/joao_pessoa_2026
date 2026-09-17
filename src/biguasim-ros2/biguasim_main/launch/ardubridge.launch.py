from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    print('Lançando o Nó ArduBridge com Namespace')

    config_dir = str(Path(get_package_share_directory('biguasim_main')) / 'config')
    # Built as a SUBSTITUTION LIST, not an f-string: agent_name is only a
    # promise to produce a string later, so formatting it here would embed the
    # object's repr in the path. The node resolves it when it starts.
    params_file = [config_dir, '/config-', LaunchConfiguration('agent_name'), '.yaml']

    biguasim_namespace = 'biguasim'

    ardubridge_node = launch_ros.actions.Node(
        name='ardubridge_node',
        package='biguasim_main',
        executable='ardubridge_node',  
        namespace=biguasim_namespace,
        output='screen',
        emulate_tty=True,
        parameters=[{
            'params_file': ParameterValue(params_file, value_type=str),
            # Which competition phase is flying. Phases 3 and 4 fly the Kopis,
            # whose Livox Mid-360 is a depth camera this node spins; 1 and 2
            # fly the Holybro, whose DepthCamera is the ZED's and must never
            # be turned. See ArduBridgeNode._spin_setup.
            'phase': ParameterValue(LaunchConfiguration('phase'), value_type=int),
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'agent_name', default_value='HolybroX500',
            description='Which airframe to spawn: reads config/config-<agent_name>.yaml.'),
        DeclareLaunchArgument(
            'phase', default_value='1',
            description='Competition phase. 3 and 4 fly the Kopis and spin a '
                        'depth camera to simulate its Livox Mid-360; 1 and 2 '
                        'fly the Holybro and spin nothing.'),
        ardubridge_node
    ])
