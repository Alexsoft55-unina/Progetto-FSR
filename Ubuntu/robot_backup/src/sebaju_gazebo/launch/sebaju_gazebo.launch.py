import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

SPAWN_HEIGHT = 0.2439  # torso height of SeBaJu_BOT(II).xml
# The Fortress contact system ignores <topic> and publishes on the scoped sensor topic.
LEFT_CONTACT_GZ_TOPIC = '/world/sebaju_world/model/sebaju/link/left_wheel_link/sensor/left_wheel_contact/contact'
RIGHT_CONTACT_GZ_TOPIC = '/world/sebaju_world/model/sebaju/link/right_wheel_link/sensor/right_wheel_contact/contact'


def generate_launch_description():
    pkg_share = get_package_share_directory('sebaju_gazebo')
    world_file = os.path.join(pkg_share, 'worlds', 'sebaju_world.sdf')
    xacro_file = os.path.join(pkg_share, 'urdf', 'sebaju.urdf.xacro')
    controllers_file = os.path.join(pkg_share, 'config', 'sebaju_controllers.yaml')

    gui = LaunchConfiguration('gui')
    jump_enable = LaunchConfiguration('jump_enable')
    jump_start_time = LaunchConfiguration('jump_start_time')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' controllers_file:=', controllers_file]),
        value_type=str,
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': PythonExpression(
                ["'", world_file, " -r -v 1' + ('' if '", gui, "'.lower() == 'true' else ' -s')"]),
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'sebaju', '-z', str(SPAWN_HEIGHT)],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/sebaju/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/sebaju/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            LEFT_CONTACT_GZ_TOPIC + '@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            RIGHT_CONTACT_GZ_TOPIC + '@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            '/sebaju/release@std_msgs/msg/Empty]gz.msgs.Empty',
        ] + [f'/sebaju/{joint}/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double'
             for joint in ('left_hip', 'right_hip', 'left_knee', 'right_knee')],
        remappings=[
            (LEFT_CONTACT_GZ_TOPIC, '/sebaju/left_wheel_contact'),
            (RIGHT_CONTACT_GZ_TOPIC, '/sebaju/right_wheel_contact'),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
    )

    effort_controllers_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['wheel_effort_controller', '--controller-manager', '/controller_manager'],
    )

    balance_jump_controller = Node(
        package='sebaju_gazebo',
        executable='balance_jump_controller',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
            'jump_enable': ParameterValue(jump_enable, value_type=bool),
            'jump_start_time': ParameterValue(jump_start_time, value_type=float),
            'drive_enable': ParameterValue(LaunchConfiguration('drive_enable'), value_type=bool),
            'drive_distance': ParameterValue(LaunchConfiguration('drive_distance'), value_type=float),
            'drive_period': ParameterValue(LaunchConfiguration('drive_period'), value_type=float),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Start the Gazebo GUI'),
        DeclareLaunchArgument('jump_enable', default_value='true', description='Perform the vertical jump'),
        DeclareLaunchArgument('jump_start_time', default_value='5.0',
                              description='Seconds after release before the jump starts'),
        DeclareLaunchArgument('drive_enable', default_value='false',
                              description='Drive back and forth on the wheels while balancing (use with jump_enable:=false)'),
        DeclareLaunchArgument('drive_distance', default_value='1.0', description='Back-and-forth travel [m]'),
        DeclareLaunchArgument('drive_period', default_value='8.0', description='Duration of one back-and-forth cycle [s]'),
        gz_sim,
        bridge,
        robot_state_publisher,
        spawn_robot,
        RegisterEventHandler(OnProcessExit(target_action=spawn_robot,
                                           on_exit=[joint_state_broadcaster_spawner])),
        RegisterEventHandler(OnProcessExit(target_action=joint_state_broadcaster_spawner,
                                           on_exit=[effort_controllers_spawner])),
        RegisterEventHandler(OnProcessExit(target_action=effort_controllers_spawner,
                                           on_exit=[balance_jump_controller])),
    ])
