import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    pkg = get_package_share_directory('ur5_rg2_ign')
    urdf_file = os.path.join(pkg, 'urdf', 'ur5_rg2.urdf')
    controllers_yaml = os.path.join(pkg, 'config', 'controllers.yaml')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    robot_description = robot_description.replace('__CONTROLLERS_YAML__', controllers_yaml)

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('gazebo_ros'),
                             'launch', 'gazebo.launch.py')
            ]),
            launch_arguments={'world': os.path.join(pkg, 'launch', 'empty_world.world'), 'gui': 'false'}.items(),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'ur5_rg2',
                '-topic', 'robot_description',
                '-x', '0.5', '-y', '0', '-z', '0.5',
            ],
        ),
        TimerAction(period=3.0, actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
            ),
        ]),
        TimerAction(period=4.0, actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_trajectory_controller'],
            ),
        ]),

        TimerAction(period=7.0, actions=[
            ExecuteProcess(
                cmd=['ros2', 'topic', 'pub', '--once',
                     '/joint_trajectory_controller/joint_trajectory',
                     'trajectory_msgs/msg/JointTrajectory',
                     '{header: {frame_id: ""}, joint_names: ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"], points: [{positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], time_from_start: {sec: 2}}]}'],
                output='screen'
            ),
        ]),
    ])
