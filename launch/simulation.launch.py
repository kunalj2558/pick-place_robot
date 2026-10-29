import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('synora_robot_arm')
    ros_gz_sim_share = FindPackageShare('ros_gz_sim')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    rviz = LaunchConfiguration('rviz', default='false')
    gz_args = LaunchConfiguration('gz_args', default='')

    # Paths
    controllers_file = PathJoinSubstitution([pkg_share, 'config', 'robot_controllers.yaml'])
    world_file = PathJoinSubstitution([pkg_share, 'worlds', 'robot_arm.world.sdf'])
    rviz_config_file = PathJoinSubstitution([pkg_share, 'config', 'robot.rviz'])
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'robot_arm.urdf.xacro'])

    # Process Xacro to URDF with controllers_file argument
    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        xacro_file,
        ' ',
        'controllers_file:=',
        controllers_file,
    ])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # 1. Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # 2. Gazebo Sim Launch
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ros_gz_sim_share, 'launch', 'gz_sim.launch.py'])
        ),
        launch_arguments=[
            ('gz_args', [gz_args, ' -r -v 1 ', world_file])
        ]
    )

    # 3. Spawn Entity in Gazebo
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-world', 'robot_arm_world',
            '-topic', 'robot_description',
            '-name', 'synora_robot_arm',
            '-allow_renaming', 'true',
            '-z', '0.0',
        ],
    )

    # 4. ROS-Gazebo Clock Bridge
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # 5. Controller Spawners
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--param-file', controllers_file,
            '--controller-manager-timeout', '60',
        ],
        output='screen'
    )

    arm_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'arm_controller',
            '--param-file', controllers_file,
            '--controller-manager-timeout', '60',
        ],
        output='screen'
    )

    gripper_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'gripper_controller',
            '--param-file', controllers_file,
            '--controller-manager-timeout', '60',
        ],
        output='screen'
    )

    # 6. Event Handlers for Sequencing Controller Activation
    # Wait for entity spawn to exit before starting joint_state_broadcaster
    start_joint_state_broadcaster_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    # Wait for joint_state_broadcaster to activate before starting arm_controller
    start_arm_controller_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    # Wait for arm_controller to activate before starting gripper_controller
    start_gripper_controller_after_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    # 7. Optional RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz)
    )

    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Start RViz2 alongside Gazebo simulation'
        ),
        DeclareLaunchArgument(
            'gz_args',
            default_value='',
            description='Additional Gazebo arguments'
        ),

        # Core nodes & actions
        clock_bridge,
        node_robot_state_publisher,
        gz_sim,
        gz_spawn_entity,

        # Sequenced controller activation
        start_joint_state_broadcaster_after_spawn,
        start_arm_controller_after_jsb,
        start_gripper_controller_after_arm,

        # Visualization
        rviz_node,
    ])
