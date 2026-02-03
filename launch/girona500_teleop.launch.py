from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_stonefish_ros2 = get_package_share_directory('stonefish_ros2')

    # Simulation parameters
    simulation_data = os.path.join(pkg_stonefish_ros2, 'data')
    scenario_desc = os.path.join(pkg_stonefish_ros2,'scenarios', 'console_test.scn')
    
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

    # GIRONA500 Teleoperation node
    teleop_node = Node(
        package='stonefish_ros2',
        executable='girona500_teleop.py',
        name='girona500_teleop',
        output='screen',
        prefix='xterm -e',  # Run in separate terminal for keyboard input
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
    ])
