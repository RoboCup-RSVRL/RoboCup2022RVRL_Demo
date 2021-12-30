
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_name = LaunchConfiguration('robot_name', default='robot1')

    param_file_name =  'p3at.yaml'
    param_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            get_package_share_directory('rvrl_navigation2'),
            'param',
            param_file_name))

    nav2_launch_file_dir = os.path.join(get_package_share_directory('rvrl_navigation2'), 'launch')

    rviz_config_dir = os.path.join(
        get_package_share_directory('rvrl_navigation2'),
        'rviz',
        'p3at_navigation2.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=param_dir,
            description='Full path to param file to load'),
        DeclareLaunchArgument(
            'robot_name',
            default_value='robot1',
            description='Robot Name'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'use_sim_time': use_sim_time,'namespace':robot_name,
                'params_file': param_dir}.items(),
        ),

        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config_dir],
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     output='screen'),
    ])
