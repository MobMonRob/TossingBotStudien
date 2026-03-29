import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('ur5_rg2_ign')
    urdf_file = os.path.join(pkg, 'urdf', 'ur5_rg2.urdf')
    srdf_file = os.path.join(pkg, 'config', 'ur5_rg2.srdf')
    kinematics_file = os.path.join(pkg, 'config', 'kinematics.yaml')
    joint_limits_file = os.path.join(pkg, 'config', 'joint_limits.yaml')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    with open(srdf_file, 'r') as f:
        robot_description_semantic = f.read()

    return LaunchDescription([
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'robot_description_semantic': robot_description_semantic,
                'robot_description_kinematics': kinematics_file,
                'robot_description_planning': joint_limits_file,
                'allow_trajectory_execution': True,
                'publish_monitored_planning_scene': True,
                'use_sim_time': True,
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg, 'launch', 'ur5_rg2_moveit.rviz')],
            parameters=[{
                'robot_description': robot_description,
                'robot_description_semantic': robot_description_semantic,
                'use_sim_time': True,
            }],
        ),
    ])
