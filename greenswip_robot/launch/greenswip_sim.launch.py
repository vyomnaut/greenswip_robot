import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg = get_package_share_directory('greenswip_robot')

    debug_vision_arg = DeclareLaunchArgument('debug_vision', default_value='false')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    debug_vision = LaunchConfiguration('debug_vision')
    use_sim_time = LaunchConfiguration('use_sim_time')

    urdf_path  = os.path.join(pkg, 'urdf',   'ack.urdf.xacro')
    world_path = os.path.join(pkg, 'worlds', 'shapes.sdf')
    bridge_cfg = os.path.join(pkg, 'config', 'bridge.yaml')

    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    gazebo = ExecuteProcess(
        cmd=['ign', 'gazebo', '-r', world_path], output='screen')

    spawn_robot = Node(
        package='ros_gz_sim', executable='create',
        arguments=['-name', 'ackerman_robot',
                   '-x', '0.0', '-y', '0.0', '-z', '0.05',
                   '-topic', '/robot_description'],
        output='screen')

    robot_state_pub = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': use_sim_time}],
        output='screen')

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        parameters=[{'config_file': bridge_cfg}],
        output='screen')

    perception = Node(
        package='greenswip_robot', executable='perception_node.py',
        name='perception_node',
        parameters=[{'use_sim_time': use_sim_time, 'show_debug': debug_vision, 'min_contour_area': 500}],
        output='screen')

    control = Node(
        package='greenswip_robot', executable='control_node.py', prefix='python3',
        name='control_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen')

    return LaunchDescription([
        debug_vision_arg, use_sim_time_arg,
        gazebo, robot_state_pub,
        TimerAction(period=3.0, actions=[spawn_robot]),
        TimerAction(period=4.0, actions=[bridge]),
        TimerAction(period=5.0, actions=[perception, control]),
    ])
