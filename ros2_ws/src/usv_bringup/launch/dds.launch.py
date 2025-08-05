import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess


def generate_launch_description():
    pppd = ExecuteProcess(
        cmd=[
            'sudo', '/usr/sbin/pppd',
            '/dev/ttyTHS1', '115200',
            '192.168.13.16:192.168.13.65',
            'crtscts', 'debug', 'noauth', 'nodetach', 'local', 'proxyarp', 'ktune'
        ],
        shell=True,
        output='screen',
    )

    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        output='screen',
        arguments=[
            'udp4',
            '-p', '2019',
        ]
    )

    ld = LaunchDescription()

    ld.add_action(pppd)
    ld.add_action(micro_ros_agent)
 
    return ld
