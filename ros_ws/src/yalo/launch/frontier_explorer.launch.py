from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    map_topic = LaunchConfiguration('map_topic')
    global_frame = LaunchConfiguration('global_frame')
    robot_frame = LaunchConfiguration('robot_frame')
    timer_period = LaunchConfiguration('timer_period')
    use_sim_time = LaunchConfiguration('use_sim_time')

    start_rosbag = LaunchConfiguration('start_rosbag')
    rosbag_uri = LaunchConfiguration('rosbag_uri')
    rosbag_rate = LaunchConfiguration('rosbag_rate')

    start_slam = LaunchConfiguration('start_slam')
    start_republish = LaunchConfiguration('start_republish')
    start_rviz = LaunchConfiguration('start_rviz')
    republish_in_topic = LaunchConfiguration('republish_in_topic')
    republish_out_topic = LaunchConfiguration('republish_out_topic')

    frontier_entropy_topic = LaunchConfiguration('frontier_entropy_topic')
    frontier_centroids_topic = LaunchConfiguration('frontier_centroids_topic')
    frontier_cluster_sizes_topic = LaunchConfiguration('frontier_cluster_sizes_topic')
    goal_topic = LaunchConfiguration('goal_topic')

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
        DeclareLaunchArgument(
            'start_rosbag',
            default_value='true',
            description='Start rosbag playback process',
        ),
        DeclareLaunchArgument(
            'rosbag_uri',
            default_value='/home/eva/ros2_ws/bags/rosbag_turtlebot_static_1/rosbag2_2026_04_11-16_12_00_0.mcap',
            description='Path or URI passed to ros2 bag play',
        ),
        DeclareLaunchArgument(
            'rosbag_rate',
            default_value='0.5',
            description='Playback rate for rosbag',
        ),
        DeclareLaunchArgument(
            'start_slam',
            default_value='true',
            description='Include turtlebot4_navigation slam.launch.py',
        ),
        DeclareLaunchArgument(
            'start_republish',
            default_value='true',
            description='Run image_transport republish compressed->raw',
        ),
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Start RViz2 with frontier config',
        ),
        DeclareLaunchArgument(
            'republish_in_topic',
            default_value='/oakd/rgb/preview/image_raw/compressed',
            description='Input compressed image topic for image_transport republish',
        ),
        DeclareLaunchArgument(
            'republish_out_topic',
            default_value='/oakd/rgb/image_raw',
            description='Output raw image topic for image_transport republish',
        ),
        DeclareLaunchArgument(
            'frontier_centroids_topic',
            default_value='/frontier_centroids',
        ),
        DeclareLaunchArgument(
            'frontier_entropy_topic',
            default_value='/frontier_entropy_scores',
        ),
        DeclareLaunchArgument(
            'frontier_cluster_sizes_topic',
            default_value='/frontier_cluster_sizes',
        ),
        DeclareLaunchArgument(
            'goal_topic',
            default_value='/goal_pose',
        ),
        ExecuteProcess(
            condition=IfCondition(start_rosbag),
            cmd=[
                'ros2',
                'bag',
                'play',
                rosbag_uri,
                '--loop',
                '--rate',
                rosbag_rate,
            ],
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('turtlebot4_navigation'),
                    'launch',
                    'slam.launch.py',
                ])
            ),
            condition=IfCondition(start_slam),
        ),
        Node(
            condition=IfCondition(start_republish),
            package='image_transport',
            executable='republish',
            name='rgb_republish_compressed_to_raw',
            output='screen',
            arguments=['compressed', 'raw'],
            remappings=[
                ('in/compressed', republish_in_topic),
                ('out', republish_out_topic),
            ],
        ),
        Node(
            package='yalo',
            executable='frontier_detector',
            name='frontier_detector',
            output='screen',
            parameters=[{
                'map_topic': map_topic,
                'global_frame': global_frame,
                'robot_frame': robot_frame,
                'timer_period': timer_period,
                'use_sim_time': use_sim_time,
                'frontier_centroids_topic': frontier_centroids_topic,
                'frontier_entropy_topic': frontier_entropy_topic,
                'frontier_cluster_sizes_topic': frontier_cluster_sizes_topic,
            }],
        ),
        Node(
            package='yalo',
            executable='decision_maker',
            name='decision_maker',
            output='screen',
            parameters=[{
                'frontier_centroids_topic': frontier_centroids_topic,
                'frontier_entropy_topic': frontier_entropy_topic,
                'frontier_cluster_sizes_topic': frontier_cluster_sizes_topic,
                'goal_topic': goal_topic,
                'odom_topic': '/odom',
                'use_sim_time': use_sim_time,
                'decision_period': 1.0,
                'entropy_weight': 1.0,
                'distance_weight': 0.25,
                'consistency_weight': 0.45,
                'switch_margin': 0.15,
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
            },
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('yalo'),
                'rviz',
                'frontier.rviz',
            ])],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
