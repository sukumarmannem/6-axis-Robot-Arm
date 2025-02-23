#!/usr/bin/env python3
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('six_axis_robot')
    urdf_file = os.path.join(pkg_share, 'urdf', 'robot_model_6_dof.urdf.xacro')
    robot_description_command = Command(['/opt/ros/humble/bin/xacro', ' ', urdf_file])
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_description',
            default_value=robot_description_command,
            description='URDF file for the robot'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(robot_description_command, value_type=str)
            }]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(robot_description_command, value_type=str)
            }]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}],
        )
    ])

if __name__ == '__main__':
    generate_launch_description()





























