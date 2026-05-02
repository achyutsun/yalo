"""
entropy_exploration.launch.py
─────────────────────────────
Launch file for YALO TurtleBot4 Entropy Explorer.

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

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution, PythonExpression
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


home = os.path.expanduser('~')
map_yaml_path = os.path.join(home, 'ros2_ws/src/yalo/resource/map.yaml') 

def generate_launch_description():

    # --- Package share directories & file paths ---
    pkg = get_package_share_directory('yalo')
    rviz = os.path.join(pkg, 'rviz', 'exploration.rviz')
    # slam_share = get_package_share_directory('slam_toolbox')
    # nav2_share = get_package_share_directory('nav2_bringup')

    # ── Config file paths ──────────────────────────────────────────────────
    slam_yaml = os.path.join(pkg, 'config', 'slam_toolbox_params.yaml')
    nav2_yaml = os.path.join(pkg, 'config', 'nav2_params.yaml')

    if not os.path.isfile(slam_yaml):
        print(f'\n[WARN] Missing: {slam_yaml}\n       Using slam_toolbox defaults.\n')
    if not os.path.isfile(nav2_yaml):
        print(f'\n[WARN] Missing: {nav2_yaml}\n       Using nav2_bringup defaults.\n')

    # ── Arguments ──────────────────────────────────────────────────────────
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use /clock (simulation time)'
    )
    slam_params_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=slam_yaml,
        description='SLAM Toolbox parameters file'
    )
    nav2_params_arg = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=nav2_yaml,
        # ([
        #     FindPackageShare('yalo'),
        #     'config', 'nav2_params.yaml'
        # ])
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
                package    = 'yalo',
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
        # RViz for visualization
        ExecuteProcess(
            cmd=['rviz2', '-d', rviz],
            output='screen'
        )
    ])
