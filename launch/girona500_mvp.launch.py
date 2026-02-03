from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from tf2_ros import StaticTransformBroadcaster
import os


def generate_launch_description():
    # Get package directories
    pkg_stonefish_ros2 = get_package_share_directory('stonefish_ros2')

    # Simulation parameters
    simulation_data = os.path.join(pkg_stonefish_ros2, 'data')
    scenario_desc = os.path.join(pkg_stonefish_ros2, 'scenarios', 'console_test.scn')

    # MVP configuration files
    control_modes_config = os.path.join(pkg_stonefish_ros2, 'config', 'girona500_control_modes.yaml')
    mission_config = os.path.join(pkg_stonefish_ros2, 'config', 'girona500_mvp_mission.yaml')
    helm_config = os.path.join(pkg_stonefish_ros2, 'config', 'girona500_helm.yaml')

    # Launch arguments
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

    # MVP Control Node
    mvp_control_node = Node(
        package='mvp_control',
        executable='mvp_control_ros_node',
        namespace='GIRONA500',
        name='mvp_control',
        parameters=[
            {'config_file': control_modes_config},
            {'tf_prefix': 'GIRONA500'},
            {'child_link_initial': 'cg_link'},
            {'world_link_initial': 'world_ned'},
            {'child_link': 'cg_link'},
            {'world_link': 'world_ned'}
        ],
        remappings=[
            ('/GIRONA500/mvp_control/thruster_command', '/GIRONA500/thruster_setpoints'),
            ('/GIRONA500/odometry', '/GIRONA500/dynamics_fixed'),
        ],
        output='screen',
    )

    # MVP Helm Node (Mission Controller)
    mvp_helm_node = Node(
        package='mvp_helm',
        executable='mvp_helm',
        namespace='GIRONA500',
        name='mvp_helm',
        parameters=[
            {'helm_config_file': helm_config},
            {'tf_prefix': 'GIRONA500'},
            {'child_link': 'base_link'},
            {'world_link': 'world_ned'},
            mission_config
        ],
        remappings=[
            ('/GIRONA500/odometry/filtered', '/GIRONA500/dynamics_fixed'),
            ('mvp_helm/setpoint_bhv', 'controller/process/set_point'),
        ],
        output='screen',
    )

    # Teleoperation node (keyboard control)
    teleop_node = Node(
        package='stonefish_ros2',
        executable='girona500_teleop.py',
        name='girona500_teleop',
        output='screen',
        prefix='xterm -e',
    )

    # Sensor monitor node
    sensor_monitor_node = Node(
        package='stonefish_ros2',
        executable='sensor_monitor.py',
        name='sensor_monitor',
        output='screen',
        prefix='xterm -e',
    )

    # Odometry to TF bridge (converts /GIRONA500/dynamics to TF)
    odom_to_tf_node = Node(
        package='stonefish_ros2',
        executable='odom_to_tf.py',
        name='odom_to_tf',
        output='screen',
    )

    # Odometry frame fixer (fixes frame IDs for MVP Control)
    odom_frame_fixer_node = Node(
        package='stonefish_ros2',
        executable='odom_frame_fixer.py',
        name='odom_frame_fixer',
        output='screen',
    )

    # Robot state publisher (publishes TF from URDF)
    urdf_file = os.path.join(pkg_stonefish_ros2, 'urdf', 'girona500.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='GIRONA500',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'frame_prefix': 'GIRONA500/'
        }]
    )

    # Static TF publishers to bridge Stonefish and MVP Control frames
    # Stonefish uses 'world_ned' and 'GIRONA500/Vehicle'
    # MVP Control expects 'GIRONA500/world_ned' and 'GIRONA500/base_link'

    static_tf_world_ned = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_ned',
        arguments=['0', '0', '0', '0', '0', '0', 'world_ned', 'GIRONA500/world_ned']
    )

    static_tf_vehicle_baselink = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_vehicle_baselink',
        arguments=['0', '0', '0', '0', '0', '0', 'GIRONA500/Vehicle', 'GIRONA500/base_link']
    )

    # Static TF from base_link to cg_link (identity - for behaviors that reference cg_link)
    static_tf_baselink_cglink = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_baselink_cglink',
        arguments=['0', '0', '0', '0', '0', '0', 'GIRONA500/base_link', 'GIRONA500/cg_link']
    )

    return LaunchDescription([
        simulation_data_arg,
        scenario_desc_arg,
        simulation_rate_arg,
        window_res_x_arg,
        window_res_y_arg,
        rendering_quality_arg,
        static_tf_world_ned,
        static_tf_vehicle_baselink,
        static_tf_baselink_cglink,
        robot_state_publisher_node,
        odom_to_tf_node,
        odom_frame_fixer_node,
        stonefish_simulator_node,
        mvp_control_node,
        mvp_helm_node,
        teleop_node,
        sensor_monitor_node,
    ])