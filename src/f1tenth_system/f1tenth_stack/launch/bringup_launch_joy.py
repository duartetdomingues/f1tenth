# MIT License

# Copyright (c) 2020 Hongrui Zheng

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.actions import ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from datetime import datetime
from launch.actions import OpaqueFunction
from launch.event_handlers import OnShutdown
import subprocess
from math import pi as PI

def save_map_on_shutdown(context, *args, **kwargs):
        map_filename = f"/home/jetson/f1tenth/maps/map_{datetime.now().strftime('%Y_%m_%d-%H_%M')}"
        subprocess.run([
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', map_filename
        ])
        return []

def generate_launch_description():
    joy_teleop_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'joy_teleop.yaml'
    )
    vesc_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'vesc.yaml'
    )
    lidar_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'lidar.yaml'
    )
    mux_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'mux.yaml'
    )

    zed_config = os.path.join(
        get_package_share_directory('f1tenth_stack'), 
        'config', 
        'zed2i.yaml'
    )

    robot_localization_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'robot_localization.yaml'
    )

    xsens_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'xsens.yaml'
    )

    slam_config_map = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'f1tenth_online_async.yaml'
    )

    slam_config_loc = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'slam_loc.yaml'
    )


    joy_la = DeclareLaunchArgument(
        'joy_config',
        default_value=joy_teleop_config,
        description='Descriptions for joy and joy_teleop configs')
    vesc_la = DeclareLaunchArgument(
        'vesc_config',
        default_value=vesc_config,
        description='Descriptions for vesc configs')
    lidar_la = DeclareLaunchArgument(
        'lidar_config',
        default_value=lidar_config,
        description='Descriptions for sensor configs')
    mux_la = DeclareLaunchArgument(
        'mux_config',
        default_value=mux_config,
        description='Descriptions for ackermann mux configs')
    
    zed_la = DeclareLaunchArgument(
        'zed_config',
        default_value=zed_config,
        description='Descriptions for zed camera configs'
    )

    xsens_la = DeclareLaunchArgument(
        'xsens_config',
        default_value=xsens_config,
        description='Descriptions for xsens imu configs'
    )

    robot_localization_la = DeclareLaunchArgument(
        'robot_localization_config',
        default_value=robot_localization_config,
        description='Descriptions for robot localization configs'
    )

    slam_la_map = DeclareLaunchArgument(
        'slam_config_map',
        default_value=slam_config_map,
        description='Descriptions for slam configs'
    )

    slam_la_loc = DeclareLaunchArgument(
        'slam_config_loc',
        default_value=slam_config_loc,
        description='Descriptions for slam configs'
    )

    ld = LaunchDescription([joy_la, vesc_la, lidar_la, mux_la, zed_la, robot_localization_la, xsens_la, slam_la_map, slam_la_loc])

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[LaunchConfiguration('joy_config')]
    )
    joy_teleop_node = Node(
        package='joy_teleop',
        executable='joy_teleop',
        name='joy_teleop',
        parameters=[LaunchConfiguration('joy_config')]
    )
    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        output='screen',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    throttle_interpolator_node = Node(
        package='f1tenth_stack',
        executable='throttle_interpolator',
        name='throttle_interpolator',
        parameters=[LaunchConfiguration('vesc_config')]
    )

    urg_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        parameters=[LaunchConfiguration('lidar_config')]
    )
    
    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[LaunchConfiguration('mux_config')],
        remappings=[('ackermann_cmd_out', 'ackermann_drive')]
    )

    static_tf_node_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_laser',
        arguments=['0.08', '0.0', '0.11', '0.0', '0.0', '0.0', 'base_link', 'laser']
    )

    static_tf_node_vesc = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_vesc',
        #arguments=['0.0', '0.0','0.05','-1.5601498','0.01884956', '0.0', 'base_link', 'vesc']
        #arguments=['0.0', '0.0','0.05','0.0', '-0.0113', '-3.1359', 'base_link', 'vesc']
        #arguments=['0.0', '0.0','0.05','-1.5601498','0.01884956', '0.0', 'base_link', 'vesc']
        #arguments=['0.0', '0.0','0.05', str(-PI/2), '0.0', '0.0', 'base_link', 'vesc'] # Baseline
        arguments=['0.0', '0.0','0.05', '-1.56880729', '0.02426539', '0.08179556', 'base_link', 'vesc'] #Calibrated on 2025-05-30
    )

    static_tf_node_xsens = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_xsens',
        #arguments=['0.0', '-0.01','0.0','-1.5640','-0.02', '3.141', 'base_link', 'xsens']
        #arguments=['0.0', '-0.01','0.0','-1.5882','0.0002','-3.1268', 'base_link', 'xsens']
        #arguments=['0.0', '-0.01','0.0','1.57090571', '-0.00560494', '3.12207931', 'base_link', 'xsens']
        #arguments=['0.0', '-0.01','0.0', str(-PI/2), '0.01232010', str(PI), 'base_link', 'xsens'] # Baseline
        arguments=['0.0', '-0.01','0.0', '-1.57087905', '0.00511603', '3.12542533', 'base_link', 'xsens'] #Calibrated on 2025-05-30
    )

    static_tf_node_zed2i = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_zed2i',
        #arguments=['0.2', '0.0','0.1','0.5','-0.5', '0.5', '-0.5', 'base_link', 'zed2i']
        #arguments=['0.2', '0.0','0.1','0.46706241', '-0.46706241',  '0.53089801', '-0.53089801', 'base_link', 'zed2i']
        #arguments=['0.2', '0.0','0.1','0.46697594', '-0.46697437', '0.52135815', '-0.54042028', 'base_link', 'zed2i']
        #arguments=['0.2', '0.0','0.1','1.5640', '0.0', '1.5640', 'base_link', 'zed2i']
        #arguments=['0.2', '0.0','0.1','1.57079633', '0.01425753', '1.41652568', 'base_link', 'zed2i'] #Calibrated on 2025-05-29
        #arguments=['0.2', '0.0','0.1', str(-PI/2), '0.0', str(-PI/2), 'base_link', 'zed2i'] # Baseline
        arguments=['0.2', '0.0','0.1', '-1.57000325', '0.00534136', '-1.42339360', 'base_link', 'zed2i'] #Calibrated on 2025-05-30
    )

    """ zed_node = Node(
       package='zed_wrapper',
       executable='zed_wrapper_node',
       name='zed_node',
       output='screen',
       parameters=[LaunchConfiguration('zed_config')]
    ) """

    robot_localization_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory("f1tenth_stack"), 'config', 'robot_localization.yaml')],
    )

    path_robot_localization_node = Node(
            package='odom_to_path',
            executable='odom_to_path',
            name='odom_to_path',
            output='screen',
            parameters=[
                {"odom_topic": "/odometry/filtered"}
            ]
    )

    xsens_node = Node(
        package='xsens_mti_ros2_driver',
        executable='xsens_mti_node',
        name='xsens_mti_node',
        parameters=[LaunchConfiguration('xsens_config')]
    )

    slam_node_map = Node(
        package='slam_toolbox',
        executable='map_and_localization_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[LaunchConfiguration('slam_config_map')]
    )

    slam_node_loc = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[LaunchConfiguration('slam_config_loc')]
    )

   
    """ zed_camera_node = Node(
        package='zed_rgb_node',
        executable='zed_rgb_node',
        name='zed_rgb_node'
    )

    zed_sdk_node = Node(
        package='zed_sdk_cpp',
        executable='zed_sdk_cpp',
        name='zed_sdk_cpp'
    ) """

    """ save_map = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                OpaqueFunction(function=save_map_on_shutdown)
            ]
        )
    )  """
    
    imu_base_link_node = Node(
        package='imu_base_link',
        executable='imu_base_link',
        name='imu_base_link',
    )

    # finalize
    #tfs
    ld.add_action(static_tf_node_laser)
    ld.add_action(static_tf_node_vesc)
    ld.add_action(static_tf_node_xsens)
    ld.add_action(static_tf_node_zed2i)

    ld.add_action(joy_node)
    ld.add_action(joy_teleop_node)
    
    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(vesc_driver_node)
    
    #ld.add_action(throttle_interpolator_node)
    
    ld.add_action(urg_node)
    
    ld.add_action(ackermann_mux_node)
   
    #ld.add_action(zed_node)
    #ld.add_action(zed_camera_node)
    #ld.add_action(zed_sdk_node)

    ld.add_action(robot_localization_node)
    #ld.add_action(path_robot_localization_node)


    ld.add_action(xsens_node)
    
    #ld.add_action(slam_node_map)
    #ld.add_action(slam_node_loc)
    #ld.add_action(save_map)
    
    ld.add_action(imu_base_link_node)

    

    return ld