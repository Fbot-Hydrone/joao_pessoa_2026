from launch import LaunchDescription
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    print('Lançando o Nó ArduBridge com Namespace')

    base = Path(get_package_share_directory('biguasim_main'))
    params_file = base / 'config' / 'config.yaml'
    
    biguasim_namespace = 'biguasim'

    ardubridge_node = launch_ros.actions.Node(
        name='ardubridge_node',
        package='biguasim_main',
        executable='ardubridge_node',  
        namespace=biguasim_namespace,
        output='screen',
        emulate_tty=True,
        parameters=[{'params_file': str(params_file)}]
    )

    return LaunchDescription([
        ardubridge_node
    ])