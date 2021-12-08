#!/usr/bin/env python3

import os
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from gazebo_msgs.srv import SpawnEntity
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace


def get_robot_dic(type, name, x, y, z, w):
    return {'type': type, 'name': name, 'x': str(x), 'y': str(y), 'z': str(z), 'w': str(w)}


def gen_robot_list():
    robot1 = get_robot_dic('p3at', 'robot1', 2.0, -1, 0.01, 0)
    robot2 = get_robot_dic('p3at', 'robot2', 3.0, -1, 0.01, -0.9)
    robot3 = get_robot_dic('p3at', 'robot3', 4.0, -1, 0.01, 0)

    return [robot1, robot2, robot3]


def generate_launch_description():
    world_file_name = 'house_world.model'

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    world = os.path.join(get_package_share_directory(
        'rvrl_gazebo'), 'worlds', world_file_name)
    world = os.path.join(get_package_share_directory(
        'rvrl_gazebo'), 'worlds', world_file_name)
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    robots = gen_robot_list()

    ld = LaunchDescription()

    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        pkg_gazebo_ros, 'launch', 'gzserver.launch.py')), launch_arguments={'world': world}.items()))
    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))))

    for robot in robots:
        urdf_path = os.path.join(get_package_share_directory(
            'rvrl_description'), 'urdf', robot['type']+'.urdf')
        ld.add_action(Node(package='robot_spawner_pkg',
                           executable='spawn_robot',
                           namespace=robot['name'],
                           arguments=['robot_' + robot['type'],
                                      robot['name'],
                                      robot['name'],
                                      robot['x'],
                                      robot['y'],
                                      robot['z'],
                                      robot['w']],

                           output='screen'))
        ld.add_action(Node(package='robot_state_publisher',
                           executable='robot_state_publisher',
                           name='robot_state_publisher',
                           namespace=robot['name'],
                           output='screen',
                           parameters=[{'use_sim_time': use_sim_time}],
                           arguments=[urdf_path]))

    return ld
