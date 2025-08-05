import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess


def generate_launch_description():
    # PPP daemon
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

	# uXRCE-DDS agent
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

    # Action server for behavior tree execution
    mission_server = Node(
        package='bt_executor',
        executable='mission_server',
        name='mission_server',
        output='screen',
    )

    # /cmd_vel to /ap/cmd_vel
    cmd_vel2ap = Node(
        package='topic_tools',
        executable='cmd_vel2ap',
        name='cmd_vel2ap',
        output='screen',
    )

    ld = LaunchDescription()

    ld.add_action(pppd)
    ld.add_action(micro_ros_agent)
    ld.add_action(mission_server)
    ld.add_action(cmd_vel2ap)
 
    return ld
