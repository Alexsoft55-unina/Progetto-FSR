import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    urdf_path = '/home/aldo/Scrivania/fsr/bipede.urdf'
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([

        ExecuteProcess(
            cmd=['ign', 'gazebo', '/home/aldo/Scrivania/fsr/mondo_pausa.sdf'],
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

        # t=5s: leg_controller e wheel_controller (non broadcaster, ancora in pausa)
        TimerAction(period=5.0, actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['leg_controller', '--controller-manager-timeout', '30'],
            ),
        ]),

        TimerAction(period=7.0, actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['wheel_controller', '--controller-manager-timeout', '30'],
            ),
        ]),

        # t=9s: avvia il PID (pubblica subito postura gambe piegate)
        TimerAction(period=9.0, actions=[
            ExecuteProcess(
                cmd=['python3', '/home/aldo/Scrivania/fsr/bilanciamento.py'],
                output='screen'
            ),
        ]),

        # t=12s: sblocca la pausa — robot inizia a cadere con gambe già piegate
        TimerAction(period=12.0, actions=[
            ExecuteProcess(
                cmd=[
                    'ign', 'service',
                    '-s', '/world/empty/control',
                    '--reqtype', 'ignition.msgs.WorldControl',
                    '--reptype', 'ignition.msgs.Boolean',
                    '--timeout', '5000',
                    '--req', 'pause: false'
                ],
                output='screen'
            ),
        ]),

        # t=14s: broadcaster DOPO lo sblocco — la sim sta girando, switch riesce
        TimerAction(period=14.0, actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager-timeout', '30'],
            ),
        ]),
    ])