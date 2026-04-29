import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    world = LaunchConfiguration('world')

    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value='',
        description='Full path to world file'
    )

    gzserver = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_factory.so', world],
        output='screen'
    )

    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )

    return LaunchDescription([
        declare_world_arg,
        gzserver,
        gzclient
    ])
