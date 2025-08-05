from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_path
import os

def generate_launch_description():
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true',
    )

    urdf_path = os.path.join(
        get_package_share_path('usv_description'),
        'urdf',
        'model.urdf.xacro'
    )
    rviz_config_path = os.path.join(
        get_package_share_path('usv_description'),
        'rviz', 
        'rviz_config.rviz'
    )
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path, " usv_name:=usv"]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    ) 
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    return LaunchDescription([
        declare_use_sim_time,

        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz2_node,
    ])