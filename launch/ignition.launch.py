from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
       ExecuteProcess(
    cmd=[
        'gz', 'sim',
        '-v', '4',
        '-r',
        '/home/s/tossingbot_ws/src/ur5_rg2_ign/worlds/empty_ign.world'
    ],
    output='screen'
),



        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-file', '/home/s/tossingbot_ws/src/ur5_rg2_ign/urdf/ur5_rg2.sdf',
                '-name', 'ur5_rg2'
            ],
            output='screen'
        ),
    ])

