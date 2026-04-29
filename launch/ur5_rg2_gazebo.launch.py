import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg = get_package_share_directory("ur5_rg2_ign")
    with open(os.path.join(pkg, "urdf", "ur5_rg2.urdf"), "r") as f:
        robot_description = f.read()
    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher", output="screen", parameters=[{"robot_description": robot_description}])
    gazebo = IncludeLaunchDescription(PythonLaunchDescriptionSource([FindPackageShare("gazebo_ros"), "/launch", "/gazebo.launch.py"]))
    spawn = Node(package="gazebo_ros", executable="spawn_entity.py", arguments=["-topic", "robot_description", "-entity", "ur5_rg2"], output="screen")
    load_jsb = ExecuteProcess(cmd=["ros2", "control", "load_controller", "--set-state", "active", "joint_state_broadcaster"], output="screen")
    load_jtc = ExecuteProcess(cmd=["ros2", "control", "load_controller", "--set-state", "active", "joint_trajectory_controller"], output="screen")
    return LaunchDescription([
        RegisterEventHandler(event_handler=OnProcessExit(target_action=spawn, on_exit=[load_jsb])),
        RegisterEventHandler(event_handler=OnProcessExit(target_action=load_jsb, on_exit=[load_jtc])),
        gazebo, rsp, spawn,
    ])