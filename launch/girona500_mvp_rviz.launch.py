from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directories
    pkg_stonefish_ros2 = get_package_share_directory('stonefish_ros2')

    # RViz config file
    rviz_config = os.path.join(pkg_stonefish_ros2, 'config', 'girona500_rviz.rviz')

    # Include the main MVP launch file
    mvp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_stonefish_ros2, 'launch', 'girona500_mvp.launch.py')
        )
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    # Robot trail publisher
    trail_publisher_node = Node(
        package='stonefish_ros2',
        executable='robot_trail_publisher.py',
        name='robot_trail_publisher',
        output='screen',
    )

    # Waypoint visualizer
    waypoint_visualizer_node = Node(
        package='stonefish_ros2',
        executable='waypoint_visualizer.py',
        name='waypoint_visualizer',
        output='screen',
    )

    return LaunchDescription([
        mvp_launch,
        rviz_node,
        trail_publisher_node,
        waypoint_visualizer_node,
    ])