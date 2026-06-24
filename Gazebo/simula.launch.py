import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    urdf_path = '/home/aldo/Scrivania/fsr/bipede.urdf'
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    brain = TimerAction(
        period=12.0,  # parte solo dopo che i controller sono attivi
        actions=[
            ExecuteProcess(
                cmd=['python3', '/home/aldo/Scrivania/fsr/bilanciamento.py'],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=['ign', 'gazebo', 'empty.sdf'],
            additional_env={'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': '/opt/ros/humble/lib'},
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-string', robot_desc, '-name', 'bipede_ruotato', '-z', '0.5'],
            output='screen'
        ),
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU'],
            output='screen'
        ),

        # STEP 1: prima il joint_state_broadcaster da solo
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=[
                        'joint_state_broadcaster',
                        '--controller-manager-timeout', '30'
                    ],
                ),
            ]
        ),

        # STEP 2: poi leg e wheel, dopo che il broadcaster è attivo
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=[
                        'leg_controller',
                        '--controller-manager-timeout', '30'
                    ],
                ),
                Node(
                    package='controller_manager',
                    executable='spawner',
                    arguments=[
                        'wheel_controller',
                        '--controller-manager-timeout', '30'
                    ],
                ),
            ]
        ),

        brain,
    ])