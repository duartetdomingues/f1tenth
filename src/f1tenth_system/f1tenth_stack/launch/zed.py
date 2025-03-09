import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    zed_config = os.path.join(get_package_share_directory('f1tenth_stack'), 'config', 'zed2i.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_name',
            default_value='zed2i',
            description='Name of the camera instance'
        ),
        DeclareLaunchArgument(
            'config_file',
            default_value=zed_config,
            description='Path to the config file'
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(zed_wrapper_dir, 'launch', 'zed_camera.launch.py')
            ),
            launch_arguments={
                'camera_name': LaunchConfiguration('camera_name'),
                'config_file': LaunchConfiguration('config_file')
            }.items()
        )
    ])
