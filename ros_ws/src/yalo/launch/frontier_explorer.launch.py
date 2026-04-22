from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_rosbag = LaunchConfiguration('start_rosbag')
    rosbag_uri = LaunchConfiguration('rosbag_uri')
    rosbag_rate = LaunchConfiguration('rosbag_rate')
    start_slam = LaunchConfiguration('start_slam')
    start_republish = LaunchConfiguration('start_republish')

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
            default_value='true',
        ),
        DeclareLaunchArgument(
            'start_rosbag',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'rosbag_uri',
            default_value='/home/eva/ros2_ws/bags/rosbag_turtlebot_static_1/rosbag2_2026_04_11-16_12_00_0.mcap',
        ),
        DeclareLaunchArgument(
            'rosbag_rate',
            default_value='0.5',
        ),
        DeclareLaunchArgument(
            'start_slam',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'start_republish',
            default_value='true',
        ),
        ExecuteProcess(
            condition=IfCondition(start_rosbag),
            cmd=[
                'ros2',
                'bag',
                'play',
                rosbag_uri,
                '--clock',
                '--loop',
                '--rate',
                rosbag_rate,
            ],
            output='screen',
        ),
        IncludeLaunchDescription(
            condition=IfCondition(start_slam),
            launch_description_source=PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('turtlebot4_navigation'),
                    'launch',
                    'slam.launch.py',
                ])
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
            }.items(),
        ),
        Node(
            condition=IfCondition(start_republish),
            package='image_transport',
            executable='republish',
            name='rgb_republish_compressed_to_raw',
            output='screen',
            arguments=['compressed', 'raw'],
            remappings=[
                ('in/compressed', '/oakd/rgb/image_raw/compressed'),
                ('out', '/oakd/rgb/image_raw'),
            ],
        ),
        Node(
            package='yalo',
            executable='frontier_detector',
            name='frontier_detector',
            output='screen',
            parameters=[{
                'map_topic': LaunchConfiguration('map_topic'),
                'global_frame': LaunchConfiguration('global_frame'),
                'robot_frame': LaunchConfiguration('robot_frame'),
                'timer_period': LaunchConfiguration('timer_period'),
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
                'odom_topic': '/odom',
                'goal_topic': '/goal_pose',
                'use_sim_time': use_sim_time,
            }],
        ),
        Node(
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
