#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, launch_configuration
from launch_ros.actions import Node
from launch.actions import GroupAction


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_name = LaunchConfiguration('robot_name')
    sdf = LaunchConfiguration('sdf')
    urdf = LaunchConfiguration('urdf')
    x = LaunchConfiguration('x', default='0.00')
    y = LaunchConfiguration('y', default='0.00')
    z = LaunchConfiguration('z', default='0.00')
    R = LaunchConfiguration('R', default='0.00')
    P = LaunchConfiguration('P', default='0.00')
    Y = LaunchConfiguration('Y', default='0.00')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    return launch.LaunchDescription([
        DeclareLaunchArgument('robot_name',
                              default_value='robot1',
                              description='robot name'),
        Node(package='gazebo_ros',
             executable='spawn_entity.py',
             output='screen',
             arguments=[
                 '-entity', robot_name,
                 '-file', sdf,
                 '-robot_namespace',robot_name,
                 '-x', x, '-y', y, '-z', z,
                 '-R', R, '-P', P, '-Y', Y]),
        Node(package='robot_state_publisher',
             executable='robot_state_publisher',
             name='robot_state_publisher',
             namespace=robot_name,
             output='screen',
             parameters=[{'use_sim_time': use_sim_time}, {'robot_description': urdf}], remappings=remappings),
        Node(package="tf2_ros",
             namespace=robot_name,
             executable="static_transform_publisher",
             name=["tf_static_base_link_",robot_name],
             arguments=["0", "0", "0", "0", "0", "0", "odom",  "base_link"]),
        Node(package="tf2_ros",
             namespace=robot_name,
             executable="static_transform_publisher",
             name=["tf_static_camera_frame_",robot_name],
             arguments=["0", "0", "0", "0", "0", "0", "base_link",  "camera_frame"]),
        Node(package="tf2_ros",
             namespace=robot_name,
             executable="static_transform_publisher",
             name=["tf_static_base_scan_",robot_name],
             arguments=["0", "0", "0", "0", "0", "0", "base_link",  "base_scan"])])
