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

    import yaml
    with open(kinematics_file, 'r') as f:
        kinematics = yaml.safe_load(f)
    with open(joint_limits_file, 'r') as f:
        joint_limits = yaml.safe_load(f)
    moveit_controllers_file = os.path.join(pkg, 'config', 'moveit_controllers.yaml')
    with open(moveit_controllers_file, 'r') as f:
        moveit_controllers = yaml.safe_load(f)
    ompl_planning_file = os.path.join(pkg, 'config', 'ompl_planning.yaml')
    with open(ompl_planning_file, 'r') as f:
        ompl_planning = yaml.safe_load(f)

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
                'robot_description_kinematics': kinematics,
                'robot_description_planning': joint_limits,
                'planning_plugin': 'ompl_interface/OMPLPlanner',
                'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints',
                'start_state_max_bounds_error': 0.1,
                'allow_trajectory_execution': True,
                'publish_monitored_planning_scene': True,
                'use_sim_time': True,
                'moveit_simple_controller_manager': moveit_controllers.get('moveit_simple_controller_manager', {}),
                'joint_trajectory_controller': moveit_controllers.get('joint_trajectory_controller', {}),
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            #arguments=['-d', os.path.join(pkg, 'launch', 'ur5_rg2_moveit.rviz')],
            parameters=[{
                'robot_description': robot_description,
                'robot_description_semantic': robot_description_semantic,
                'robot_description_kinematics': kinematics,
                'use_sim_time': True,
            }],
        ),
    ])
