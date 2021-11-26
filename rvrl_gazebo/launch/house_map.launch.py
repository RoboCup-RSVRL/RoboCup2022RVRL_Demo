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


def gen_robot_list():

    robots = []
    robots.append({'type': 'p3at', 'name': 'robot1', 'x': str(
        2.0), 'y': str(-1), 'z': str(0.01), 'w': str(0)})

    robots.append({'type': 'p3at', 'name': 'robot2', 'x': str(
        3.0), 'y': str(-1), 'z': str(0.01), 'w': str(0)})

    robots.append({'type': 'p3at', 'name': 'robot3', 'x': str(
        4.0), 'y': str(-1), 'z': str(0.01), 'w': str(0)})

    return robots


def generate_launch_description():
    world_file_name = 'house_world.model'

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    world = os.path.join(get_package_share_directory(
        'rvrl_gazebo'), 'worlds', world_file_name)
    world = os.path.join(get_package_share_directory(
        'rvrl_gazebo'), 'worlds', world_file_name)
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    rvrl_cartographer_path = os.path.join(
        get_package_share_directory('rvrl_cartographer'), 'launch')

    robots = gen_robot_list()

    ld = LaunchDescription()

    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        pkg_gazebo_ros, 'launch', 'gzserver.launch.py')), launch_arguments={'world': world}.items()))
    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py'))))

    for robot in robots:
        urdf_path = os.path.join(get_package_share_directory(
            'rvrl_description'), 'urdf', robot['type']+'.urdf')

        ld.add_action(GroupAction(
            actions=[PushRosNamespace(robot['name']),
                     Node(package='robot_spawner_pkg',
                          executable='spawn_robot',
                          arguments=['robot_' + robot['type'],
                                     robot['name'],
                                     robot['name'],
                                     robot['x'],
                                     robot['y'],
                                     robot['z']],
                          output='screen'),
                     Node(package='robot_state_publisher',
                          executable='robot_state_publisher',
                          name='robot_state_publisher',
                          output='screen',
                          parameters=[{'use_sim_time': use_sim_time}],
                          arguments=[urdf_path]),
                    #  Node(package="tf2_ros",
                    #       name='tf_static_odom_base',
                    #       executable="static_transform_publisher",
                    #       arguments=["0", "0", "0", "0", "0", "0",  robot['name']+"/base_link", robot['name']+"/odom"]),

                     ]
        ))

        # ld.add_action(GroupAction(
        #     actions=[PushRosNamespace(robot['name']), Node(
        #         package='robot_state_publisher',
        #         executable='robot_state_publisher',
        #         name='robot_state_publisher',
        #         output='screen',
        #         parameters=[{'use_sim_time': use_sim_time}],
        #         arguments=[urdf_path]),
        #     ]
        # ))

        # ld.add_action(GroupAction(
        #     actions=[PushRosNamespace(robot['name']), Node(
        #         package="tf2_ros",
        #         executable="static_transform_publisher",
        #         arguments = ["0", "0", "0", "0", "0", "0", "odom", "base_link"]),
        #     ]
        # ))

    return ld
