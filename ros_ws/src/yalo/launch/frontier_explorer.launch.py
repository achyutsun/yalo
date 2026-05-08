from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_topic',
            default_value='/map',
        ),
        DeclareLaunchArgument(
            'global_frame',
            default_value='map',
        ),
        DeclareLaunchArgument(
            'robot_frame',
            default_value='base_link',
        ),
        DeclareLaunchArgument(
            'timer_period',
            default_value='1.0',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
        ),
        Node(
            package='frontier_explorer',
            executable='frontier_detector',
            name='frontier_detector',
            output='screen',
            parameters=[{
                'map_topic': LaunchConfiguration('map_topic'),
                'global_frame': LaunchConfiguration('global_frame'),
                'robot_frame': LaunchConfiguration('robot_frame'),
                'timer_period': LaunchConfiguration('timer_period'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
