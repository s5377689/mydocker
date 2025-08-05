from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command
from ament_index_python.packages import get_package_share_path
from launch.substitutions import LaunchConfiguration

import os
import yaml


def replace_world_name(input_file, output_file, world_name):
    with open(input_file, 'r') as file:
        yaml_text = file.read()
    
    yaml_text = yaml_text.replace(
        "world/empty", "world/" + world_name
    )
    data = yaml.safe_load(yaml_text)

    with open(output_file, 'w') as file:
        yaml.dump(data, file, default_flow_style=False)


def generate_launch_description():
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation clock (published to /clock) if true',
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    urdf_path = os.path.join(
        get_package_share_path('usv_description'),
        'urdf',
        'model.urdf.xacro' 
    )
    usv1_robot_description = ParameterValue(
        Command(['xacro ', urdf_path,
                 ' usv_name:=usv1',
                 ' fdm_port_in:=9002',
        ]),
        value_type=str
    )
    usv2_robot_description = ParameterValue(
        Command(['xacro ', urdf_path,
                 ' usv_name:=usv2',
                 ' fdm_port_in:=9012',
        ]),
        value_type=str
    )
    remappings = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
    ]
    usv1_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace="usv1",
        parameters=[{
            'robot_description': usv1_robot_description,
            'use_sim_time': use_sim_time,
        }],
        remappings=remappings,
    )
    usv2_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace="usv2",
        parameters=[{
            'robot_description': usv2_robot_description,
        }],
        remappings=remappings,
    )

    # Ardupilot SITL
    sitl_usv1 = ExecuteProcess(
        cmd=['sim_vehicle.py', '-v', 'rover',
             '-f', 'motorboat-skid', '--model', 'json',
             '--sysid', '1', '-I0',
             '--udp', '--no-mavproxy',
             '--enable-DDS', 
            #  '-l', '24.5758890,121.9272874,0,0',
             '-l', '24.5212829,121.9240179,0,0',
            #  '-GD',
            ]
    )
    sitl_usv2 = ExecuteProcess(
        cmd=['sim_vehicle.py', '-v', 'rover',
             '-f', 'motorboat-skid', '--model', 'json',
             '--sysid', '2', '-I1',
             '--udp', '--no-mavproxy',
            #  '-l', '24.5758890,121.9272874,0,0',
             '-l', '24.5212829,121.9240179,0,0',
            ]
    )
    delayed_sitl_usv2 = TimerAction(
        period=1.1,
        actions=[sitl_usv2,]
    )
    mavproxy1 = ExecuteProcess(
        cmd=['mavproxy.py', '--master', 'udp:127.0.0.1:5760',
             '--out', 'udp:127.0.0.1:14550',
             '--out', 'udp:127.0.0.1:12345',
            ]
    )
    mavproxy2 = ExecuteProcess(
        cmd=['mavproxy.py', '--master', 'udp:127.0.0.1:5770',
             '--out', 'udp:127.0.0.1:14560',
             '--out', 'udp:127.0.0.1:12355',
            ]
    )

    # Use Ardupilot DDS Interface
    uxr_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        output='screen',
        arguments=['udp4', '-p', '2019'],
    )

    # Transform Twist message from /cmd_vel to TwistStamped for /ap/cmd_vel
    cmd_vel2ap = Node(
        package='topic_tools',
        executable='cmd_vel2ap',
        output='screen',
    )

    # Gazebo
    world_name = 'maritime'
    world_sdf = os.path.join(
        get_package_share_path('usv_bringup'),
        'worlds',
        f'{world_name}.sdf' 
    )
    gazebo_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', world_sdf],
        output='screen',
    )
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_path('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ), 
        launch_arguments={
            'gz_args': ['-v4 -g ']
        }.items()
    )
    usv1_spawn_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/usv1/robot_description",
            "-x", "-950",     
            "-y", "0",
            "-z", "2",
            "-Y", "100",
        ]
    )
    usv2_spawn_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-topic", "/usv2/robot_description",
            "-x", "10",     
            "-y", "15",
            "-z", "2",
            "-Y", "100",
        ]
    )

    # ROS2-Gazebo bridge
    gz_bridge_yaml_path = os.path.join(
        get_package_share_path('usv_bringup'),
        'config',
        'gazebo_bridge_two.yaml'
    )
    replaced_gz_bridge_yaml_path = os.path.join(
        get_package_share_path('usv_bringup'),
        'config',
        'replaced_gazebo_bridge.yaml'
    )
    replace_world_name(
        gz_bridge_yaml_path,
        replaced_gz_bridge_yaml_path,
        world_name
    )
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': replaced_gz_bridge_yaml_path
        }]
    )

    # QGC Path
    QGC_dir = '/home/user/Downloads'
    QGC_filename = 'QGroundControl.AppImage'
    QGC_filepath = os.path.join(
        QGC_dir,
        QGC_filename
    )
    qgc = ExecuteProcess(
        cmd=[QGC_filepath]
    )

    ap_odom_publisher = Node(
        package='usv_helpers',
        executable='ap_odom_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
    )   

    # Behavior tree and action servers
    mission_server = Node(
        package='bt_executor',
        executable='mission_server',
        output='screen',
    )

    # Behavior tree visualizer
    #bviz = Node(
    #    package='bt_client',
    #    executable='bviz',
    #    output='screen',
    #)

    # USV camera published image decoder
    img_raw = Node(
        package='topic_tools',
        executable='img_decoder',
        output='screen',
        parameters=[{
            'topic_name': '/camera/compressed/image_raw',
        }]
    ) 

    # ==========================================

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    
    ld.add_action(sitl_usv1)
    ld.add_action(delayed_sitl_usv2)
    ld.add_action(mavproxy1)
    ld.add_action(mavproxy2)
    ld.add_action(usv1_state_publisher_node)
    ld.add_action(usv2_state_publisher_node)

    ld.add_action(uxr_agent)
    ld.add_action(cmd_vel2ap)

    ld.add_action(qgc)

    ld.add_action(gazebo_server)
    ld.add_action(gazebo_client)

    ld.add_action(usv1_spawn_node)
    ld.add_action(usv2_spawn_node)
    ld.add_action(ros_gz_bridge)

    ld.add_action(mission_server)
    #ld.add_action(bviz)

    ld.add_action(img_raw)

    return ld
