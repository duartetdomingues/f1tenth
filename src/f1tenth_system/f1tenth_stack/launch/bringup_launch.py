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
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

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

    slam_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'slam_loc.yaml'
    )

    mpc_config = os.path.join(
        get_package_share_directory('f1tenth_stack'),
        'config',
        'mpc.yaml'
    )
    print("get_package_share_directory(f1tenth_stack): ", get_package_share_directory('f1tenth_stack'))


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

    slam_la = DeclareLaunchArgument(
        'slam_config',
        default_value=slam_config,
        description='Descriptions for slam configs'
    )

    mpc_la = DeclareLaunchArgument(
        'mpc_config',
        default_value=mpc_config,
        description='Descriptions for mpc configs'
    )

    ld = LaunchDescription([joy_la, vesc_la, lidar_la, mux_la, zed_la, robot_localization_la, xsens_la, slam_la, mpc_la])

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
        arguments=['0.0', '0.0','0.05','-1.5708','0.0', '0.0', 'base_link', 'vesc']
    )

    static_tf_node_xsens = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_baselink_to_vesc',
        arguments=['0.0', '-0.01','0.0','-1.5708','0.0', '3.141', 'base_link', 'xsens']
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

    slam_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[LaunchConfiguration('slam_config')]
    )

    mpc_node = Node(
        package='mpc',
        executable='mpc',
        name='mpc',
        parameters=[LaunchConfiguration('mpc_config')]
    )

   

    zed_camera_node = Node(
        package='zed_rgb_node',
        executable='zed_rgb_node',
        name='zed_rgb_node'
    )

    zed_sdk_node = Node(
        package='zed_sdk_cpp',
        executable='zed_sdk_cpp',
        name='zed_sdk_cpp'
    )
 
    

    # finalize
    #tfs
    ld.add_action(static_tf_node_laser)
    ld.add_action(static_tf_node_vesc)
    ld.add_action(static_tf_node_xsens)

    #ld.add_action(joy_node)
    #ld.add_action(joy_teleop_node)
    
    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(vesc_driver_node)

    #ld.add_action(throttle_interpolator_node)

    ld.add_action(urg_node)
    ld.add_action(ackermann_mux_node)

    #ld.add_action(zed_node)
    #ld.add_action(zed_camera_node)
    ld.add_action(zed_sdk_node)

    ld.add_action(robot_localization_node)
    ld.add_action(path_robot_localization_node)

    ld.add_action(xsens_node)
    
    ld.add_action(slam_node)

    ld.add_action(mpc_node)

    


    return ld