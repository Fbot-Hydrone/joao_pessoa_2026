import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # O ROS 2 descobre a pasta do ArduPilot automaticamente (substitui o comando de terminal)
    sitl_pkg = get_package_share_directory('ardupilot_sitl')
    holybro_parm_path = os.path.join(os.path.expanduser('~'), 'Documents', 'joao_pessoa_2026', 'holybro.parm')

    return LaunchDescription([
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=['udp4', '--port', '2019']
        ),
        
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('biguasim_main'), 'launch', 'ardubridge.launch.py')
            )
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sitl_pkg, 'launch', 'sitl_dds_udp.launch.py')
            ),
            launch_arguments={
                'transport': 'udp4',
                'refs': os.path.join(sitl_pkg, 'config', 'dds_xrce_profile.xml'),
                'synthetic_clock': 'True',
                'wipe': 'True',
                'model': 'JSON',
                'speedup': '1',
                'slave': '0',
                'instance': '0',
                'defaults': f"{holybro_parm_path},{os.path.join(sitl_pkg, 'config', 'default_params', 'dds_udp.parm')}",
                'sim_address': '127.0.0.1',
                'master': 'tcp:127.0.0.1:5760',
                'sitl': '127.0.0.1:5501'
            }.items()
        ),


    ])