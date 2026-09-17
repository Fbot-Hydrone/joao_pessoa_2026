## BiguaSim launch file 
# Author: Matheus G. Mateus

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    print('Launching BiguaSim Vehicle Simulation')

    config_dir = str(Path(get_package_share_directory('biguasim_main')) / 'config')
    # One scenario file per airframe; a substitution list so agent_name is
    # resolved by the node rather than formatted in as an object repr.
    params_file = [config_dir, '/config-', LaunchConfiguration('agent_name'), '.yaml']

    # List contents of the directory to debug
    
    biguasim_namespace = 'biguasim'

    biguasim_main_node = launch_ros.actions.Node(
        name='biguasim_node',
        package='biguasim_main',
        executable='biguasim_node',  
        namespace=biguasim_namespace,
        output='screen',
        emulate_tty=True,
        parameters=[{'params_file': ParameterValue(params_file, value_type=str)}],
        remappings=[
                ('/biguasim/ControlCommand', '/control_command'),
            ]    
        )


    return LaunchDescription([
        DeclareLaunchArgument(
            'agent_name', default_value='HolybroX500',
            description='Which airframe to spawn: reads config/config-<agent_name>.yaml.'),
        biguasim_main_node
            # rosbag                            
    ])

