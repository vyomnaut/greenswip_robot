"""
shuffle_objects.launch.py
=========================
Teleports the 4 world objects to a new random arrangement.
Run this WHILE the simulation is live to test robustness.

Usage:
  ros2 launch greenswip_robot shuffle_objects.launch.py arrangement:=2

arrangement:
  1  → box far-left,   sphere far-right, capsule near,  cylinder mid
  2  → box near-right, sphere far-left,  capsule far,   cylinder left
  3  → box mid-centre, sphere right,     capsule left,  cylinder far-right
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


ARRANGEMENTS = {
    '1': {
        'target_box':     '2.5  1.5 0.15',
        'decoy_sphere':   '3.0 -1.0 0.20',
        'decoy_capsule':  '1.8  0.0 0.25',
        'decoy_cylinder': '2.8  0.5 0.15',
    },
    '2': {
        'target_box':     '2.0 -0.5 0.15',
        'decoy_sphere':   '3.2  1.8 0.20',
        'decoy_capsule':  '3.0  2.5 0.25',
        'decoy_cylinder': '2.5 -1.5 0.15',
    },
    '3': {
        'target_box':     '3.0  0.0 0.15',
        'decoy_sphere':   '2.5  1.0 0.20',
        'decoy_capsule':  '2.5 -1.0 0.25',
        'decoy_cylinder': '3.5  1.5 0.15',
    },
}


def generate_launch_description():
    arr_arg = DeclareLaunchArgument(
        'arrangement', default_value='1',
        description='Object layout (1, 2, or 3)')

    arr = LaunchConfiguration('arrangement')
    actions = [arr_arg]

    for arr_id, positions in ARRANGEMENTS.items():
        for model_name, pose in positions.items():
            x, y, z = pose.split()
            cmd = Node(
                package='ros_gz_sim',
                executable='set_pose',
                arguments=[
                    '--world', 'interview_world',
                    '--name', model_name,
                    '--pose', f'{x} {y} {z} 0 0 0'
                ],
                output='screen',
                condition=IfCondition(
                    f"$(eval '{arr_id}' == '$(var arrangement)')")
            )
            actions.append(cmd)

    return LaunchDescription(actions)
