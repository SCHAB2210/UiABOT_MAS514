import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process the URDF file with xacro
    pkg_path = os.path.join(get_package_share_directory('robot_description'))
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_config.toxml(), 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # cmd_vel_listener node to publish odometry and joint states
    node_cmd_vel_listener = Node(
        package='my_listener_package',
        executable='cmd_vel_listener',
        name='cmd_vel_listener',
        output='screen'
    )

    # Static transform publisher for odom to base_link
    node_odom_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
    )

    # Joint State Publisher GUI (optional, good for debugging)
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    # Launch RViz with the specified configuration
    rviz_config_file = os.path.join(pkg_path, 'config', 'uiabot_config.rviz')
    launch_rviz = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config_file],
        output='screen'
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        node_robot_state_publisher,
        node_cmd_vel_listener,
        node_odom_to_base_link,
        node_joint_state_publisher_gui,
        launch_rviz,
    ])

