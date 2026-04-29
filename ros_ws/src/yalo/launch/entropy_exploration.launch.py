"""
entropy_exploration.launch.py
─────────────────────────────
Launch file for TurtleBot4 Entropy Explorer.

Starts:
  1. slam_toolbox  (async SLAM — builds the /map)
  2. nav2_bringup  (Nav2 navigation stack)
  3. entropy_explorer  (this package — selects frontier goals by entropy)

Usage:
  ros2 launch turtlebot4_entropy_explorer entropy_exploration.launch.py

Optional arguments:
  use_sim_time:=true    (for Gazebo / simulation)
  slam_params_file:=<path>
  nav2_params_file:=<path>
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution, PythonExpression
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Arguments ──────────────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use /clock (simulation time)'
    )
    slam_params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('turtlebot4_entropy_explorer'),
            'config', 'slam_toolbox_params.yaml'
        ]),
        description='SLAM Toolbox parameters file'
    )
    nav2_params_arg = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('turtlebot4_entropy_explorer'),
            'config', 'nav2_params.yaml'
        ]),
        description='Nav2 navigation parameters file'
    )

    use_sim_time  = LaunchConfiguration('use_sim_time')
    slam_params   = LaunchConfiguration('slam_params_file')
    nav2_params   = LaunchConfiguration('nav2_params_file')

    # ── SLAM Toolbox (online async) ────────────────────────────────────────
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch', 'online_async_launch.py'
            ])
        ]),
        launch_arguments={
            'slam_params_file': slam_params,
            'use_sim_time'    : use_sim_time,
        }.items()
    )

    # ── Nav2 Navigation Stack ──────────────────────────────────────────────
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch', 'navigation_launch.py'
            ])
        ]),
        launch_arguments={
            'params_file' : nav2_params,
            'use_sim_time': use_sim_time,
        }.items()
    )

    # ── Entropy Explorer node ──────────────────────────────────────────────
    # Delayed start — wait for SLAM + Nav2 to initialise
    entropy_node = TimerAction(
        period=8.0,   # seconds — allow SLAM & Nav2 to come up
        actions=[
            Node(
                package    = 'turtlebot4_entropy_explorer',
                executable = 'entropy_explorer',
                name       = 'entropy_explorer',
                output     = 'screen',
                parameters = [
                    {'use_sim_time'      : use_sim_time},
                    {'min_frontier_size' : 5},
                    {'sensor_range_m'    : 6.0},
                    {'lambda_decay'      : 0.35},
                    {'goal_reached_dist' : 0.35},
                    {'replanning_period' : 4.0},
                    {'stuck_timeout'     : 25.0},
                    {'use_nav2_simple'   : True},
                ],
                remappings = [
                    ('/map',       '/map'),
                    ('/odom',      '/odom'),
                    ('/goal_pose', '/goal_pose'),
                ],
            )
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_params_arg,
        nav2_params_arg,
        slam_launch,
        nav2_launch,
        entropy_node,
    ])
