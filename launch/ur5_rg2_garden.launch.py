from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
import os

def generate_launch_description():

    urdf_path = os.path.join(
        os.getenv("HOME"),
        "tossingbot_ws",
        "src",
        "ur5_rg2_ign",
        "urdf",
        "ur5_rg2.urdf"
    )

    empty_world = "/usr/share/gz/gz-sim7/worlds/empty.sdf"

    return LaunchDescription([

        # Start Gazebo Garden
        ExecuteProcess(
            cmd=["gz", "sim", empty_world, "-v", "4", "-r"],
            output="screen"
        ),

        # Wait until Garden is fully ready
        TimerAction(
            period=60.0,
            actions=[
                Node(
                    package="ros_gz_sim",
                    executable="create",
                    arguments=[
                        "-world", "empty",
                        "-name", "ur5_rg2",
                        "-file", urdf_path
                    ],
                    output="screen"
                )
            ]
        ),

        # Publish TF + joint states
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": open(urdf_path).read()}]
        ),
    ])



