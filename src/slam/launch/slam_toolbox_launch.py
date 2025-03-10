import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    slam_config = os.path.join(
        get_package_share_directory('slam'),
        'config',
        'slam_toolbox_config.yaml'
    )



    return LaunchDescription([
        # Declare a utilização de tempo de simulação
        DeclareLaunchArgument('slam_config', default_value=slam_config, description='Slam Config'),
        
        # Iniciando o nó do SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='map_and_localization_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[LaunchConfiguration('slam_config')],  # Carrega o arquivo YAML de configuração
        ),
    ])
