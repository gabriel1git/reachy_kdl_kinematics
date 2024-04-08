import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name = 'reachy.urdf.xml'
    urdf = os.path.join(
        get_package_share_directory('reachy_kdl_kinematics'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[urdf]),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            arguments=[urdf]),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            arguments=[urdf]),
        Node(
            package='reachy_kdl_kinematics',
            executable='ikine',
            name='controller',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen'),
    ])