from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_rviz = LaunchConfiguration('start_rviz')
    start_frontier_detector = LaunchConfiguration('start_frontier_detector')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_topic',
            default_value='/map',
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/odom',
        ),
        DeclareLaunchArgument(
            'goal_topic',
            default_value='/goal_pose',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
        ),
        DeclareLaunchArgument(
            'start_rviz',
            default_value='false',
        ),
        DeclareLaunchArgument(
            'start_frontier_detector',
            default_value='false',
        ),
        Node(
            condition=IfCondition(start_frontier_detector),
            package='yalo',
            executable='frontier_detector',
            name='frontier_detector',
            output='screen',
            parameters=[{
                'map_topic': LaunchConfiguration('map_topic'),
                'global_frame': 'map',
                'robot_frame': 'base_link',
                'timer_period': 1.0,
                'use_sim_time': use_sim_time,
            }],
        ),
        Node(
            package='yalo',
            executable='decision_maker',
            name='decision_maker',
            output='screen',
            parameters=[{
                'map_topic': LaunchConfiguration('map_topic'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'goal_topic': LaunchConfiguration('goal_topic'),
                'use_sim_time': use_sim_time,
            }],
        ),
        Node(
            condition=IfCondition(start_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            additional_env={
                'LIBGL_ALWAYS_SOFTWARE': '1',
                'QT_OPENGL': 'software',
            },
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('yalo'),
                'rviz',
                'frontier.rviz',
            ])],
        ),
    ])