#!/usr/bin/env python3

import os
import launch
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from gazebo_msgs.srv import SpawnEntity
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace


def gen_robot_list():
    robot1 = get_robot_dic('p3at', 'robot1', -70, -95, 0.03, 0, 0, 0)
    robot2 = get_robot_dic('p3at', 'robot2', 0.0, 0.0, 0.03, 0, 0, 0)
    robot3 = get_robot_dic('p3at', 'robot3', -70, 90, 0.03, 0, 0, 0)
    robot4 = get_robot_dic('p3at', 'robot4', 125, 20, 0.01, 0, 0, 0)
    return [robot1, robot2, robot3,robot4]


def generate_launch_description():
    robots = gen_robot_list()

    ld = spawn_world('Robocup2019_pre1-2.model')

    for robot in robots:
        ld.add_action(spawn_robot(robot))
    return ld


def get_robot_dic(type, name, x, y, z, roll, pitch, yaw):
    return {'type': type, 'name': name, 'x': str(x), 'y': str(y), 'z': str(z), 'roll': str(roll), 'pitch': str(pitch), 'yaw': str(yaw)}


def spawn_world(world_file_name):
    world = os.path.join(get_package_share_directory(
        'rvrl_gazebo'), 'worlds', world_file_name)

    ld = LaunchDescription()

    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')), launch_arguments={'world': world}.items()))

    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py'))))
    return ld


def spawn_robot(robot):
    urdf = os.path.join(get_package_share_directory(
        'rvrl_description'), 'urdf', robot['type'], 'model.urdf')
    sdf = os.path.join(get_package_share_directory(
        'rvrl_gazebo'), 'models', robot['type'], 'model.sdf')

    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    new_robot = GroupAction([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('rvrl_gazebo'),
                                                   'launch',
                                                   'robot_spawner.launch.py')),
        launch_arguments={'robot_name': TextSubstitution(text=robot['name']),
                          'sdf': sdf,
                          'urdf': robot_description,
                          'x': TextSubstitution(text=str(robot['x'])),
                          'y': TextSubstitution(text=str(robot['y'])),
                          'z': TextSubstitution(text=str(robot['z'])),
                          'R': TextSubstitution(text=str(robot['roll'])),
                          'P': TextSubstitution(text=str(robot['pitch'])),
                          'Y': TextSubstitution(text=str(robot['yaw'])), 'use_sim_time': 'true', }.items())])
    return new_robot
