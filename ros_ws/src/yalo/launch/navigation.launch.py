from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # ✅ Use TurtleBot4's working Nav2 launch
    tb4_nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot4_navigation'),
                'launch',
                'nav2.launch.py'
            )
        )
    )

    # ✅ Delay your node (Nav2 needs time)
    frontier_nav_node = TimerAction(
        period=15.0,
        actions=[
            Node(
                package='yalo',
                executable='navigation',
                name='frontier_navigator',
                output='screen',
                emulate_tty=True
            )
        ]
    )

    return LaunchDescription([
        tb4_nav_launch,
        frontier_nav_node
    ])