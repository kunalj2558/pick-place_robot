import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('synora_robot_arm')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    rviz_config_file = PathJoinSubstitution([pkg_share, 'config', 'robot.rviz'])
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'robot_arm.urdf.xacro'])
    controllers_file = PathJoinSubstitution([pkg_share, 'config', 'robot_controllers.yaml'])

    # Robot Description
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        xacro_file,
        ' ',
        'controllers_file:=',
        controllers_file,
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),
        rviz_node,
    ])
