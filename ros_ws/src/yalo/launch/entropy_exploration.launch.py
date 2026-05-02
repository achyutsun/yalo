from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use /clock (simulation time)',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')

    entropy_node = Node(
        package='yalo',
        executable='entropy_explorer',
        name='entropy_explorer',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'min_frontier_size': 5},
            {'sensor_range_m': 6.0},
            {'lambda_decay': 0.35},
            {'goal_reached_dist': 0.35},
            {'replanning_period': 4.0},
            {'stuck_timeout': 25.0},
            {'use_nav2_simple': False},
            {'publish_goal': False},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        entropy_node,
    ])
