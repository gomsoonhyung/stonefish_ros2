from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_stonefish_ros2 = get_package_share_directory('stonefish_ros2')

    # Simulation parameters
    simulation_data = os.path.join(pkg_stonefish_ros2, 'data')
    scenario_desc = os.path.join(pkg_stonefish_ros2, 'scenarios', 'custom_auv.scn')

    simulation_data_arg = DeclareLaunchArgument(
        'simulation_data',
        default_value=simulation_data
    )

    scenario_desc_arg = DeclareLaunchArgument(
        'scenario_desc',
        default_value=scenario_desc
    )

    simulation_rate_arg = DeclareLaunchArgument(
        'simulation_rate',
        default_value='100.0'
    )

    window_res_x_arg = DeclareLaunchArgument(
        'window_res_x',
        default_value='1280'
    )

    window_res_y_arg = DeclareLaunchArgument(
        'window_res_y',
        default_value='1000'
    )

    rendering_quality_arg = DeclareLaunchArgument(
        'rendering_quality',
        default_value='high'
    )

    # Stonefish simulator node
    stonefish_simulator_node = Node(
        package='stonefish_ros2',
        executable='stonefish_simulator',
        namespace='stonefish_ros2',
        name='stonefish_simulator',
        arguments=[
            LaunchConfiguration('simulation_data'),
            LaunchConfiguration('scenario_desc'),
            LaunchConfiguration('simulation_rate'),
            LaunchConfiguration('window_res_x'),
            LaunchConfiguration('window_res_y'),
            LaunchConfiguration('rendering_quality')
        ],
        output='screen',
    )

    # Custom AUV Teleoperation node (separate terminal for keyboard input)
    teleop_node = Node(
        package='stonefish_ros2',
        executable='custom_auv_teleop.py',
        name='custom_auv_teleop',
        output='screen',
        prefix='xterm -e',
    )

    # Custom AUV Sensor Monitor node (separate terminal for display)
    sensor_monitor_node = Node(
        package='stonefish_ros2',
        executable='custom_auv_sensor_monitor.py',
        name='custom_auv_sensor_monitor',
        output='screen',
        prefix='xterm -e',
    )

    return LaunchDescription([
        simulation_data_arg,
        scenario_desc_arg,
        simulation_rate_arg,
        window_res_x_arg,
        window_res_y_arg,
        rendering_quality_arg,
        stonefish_simulator_node,
        teleop_node,
        sensor_monitor_node,
    ])