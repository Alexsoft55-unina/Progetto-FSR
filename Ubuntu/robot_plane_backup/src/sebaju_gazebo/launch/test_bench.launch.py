"""
SeBaJu test bench: runs the full Gazebo balance/jump simulation together with
the CSV telemetry logger, and (by default) opens PlotJuggler pre-loaded with
config/sebaju_plotjuggler_layout.xml, which already lays out the most
significant signals (CoM lean/rate, position/velocity, wheel command vs.
ground force, height and wheel clearance) and starts streaming them live from
/sebaju/debug. PlotJuggler will still ask to confirm "Start Streaming?" once
on open (its own safety prompt, not skippable from the command line) -
click Yes and all four tabs start plotting immediately. The finished CSV file
can also be dragged in afterwards (the logger prints its exact path on
shutdown).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('sebaju_gazebo')

    gui = LaunchConfiguration('gui')
    jump_enable = LaunchConfiguration('jump_enable')
    jump_start_time = LaunchConfiguration('jump_start_time')
    output_dir = LaunchConfiguration('output_dir')
    start_plotjuggler = LaunchConfiguration('start_plotjuggler')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'sebaju_gazebo.launch.py')),
        launch_arguments={
            'gui': gui,
            'jump_enable': jump_enable,
            'jump_start_time': jump_start_time,
            'drive_enable': LaunchConfiguration('drive_enable'),
            'drive_distance': LaunchConfiguration('drive_distance'),
            'drive_period': LaunchConfiguration('drive_period'),
            'planar_enable': LaunchConfiguration('planar_enable'),
            'traj_length': LaunchConfiguration('traj_length'),
            'traj_lateral': LaunchConfiguration('traj_lateral'),
            'traj_duration': LaunchConfiguration('traj_duration'),
            'start_controller': LaunchConfiguration('start_controller'),
        }.items(),
    )

    logger = Node(
        package='sebaju_gazebo',
        executable='test_bench_logger',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': True,
            'output_dir': output_dir,
        }],
    )

    plotjuggler_layout = os.path.join(pkg_share, 'config', 'sebaju_plotjuggler_layout.xml')

    plotjuggler = ExecuteProcess(
        cmd=['ros2', 'run', 'plotjuggler', 'plotjuggler',
             '-l', plotjuggler_layout,
             '--start_streamer', 'ROS2 Topic Subscriber'],
        output='screen',
        condition=IfCondition(start_plotjuggler),
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
        DeclareLaunchArgument('planar_enable', default_value='false',
                              description='Follow the planar polynomial S-trajectory while balancing (use with jump_enable:=false)'),
        DeclareLaunchArgument('traj_length', default_value='3.0', description='Trajectory arc length [m]'),
        DeclareLaunchArgument('traj_lateral', default_value='0.6', description='Lateral offset of the S-curve [m]'),
        DeclareLaunchArgument('traj_duration', default_value='12.0', description='Trajectory duration [s]'),
        DeclareLaunchArgument('start_controller', default_value='true',
                              description='Start balance_jump_controller here; set false to launch it '
                                          'by hand from another terminal instead'),
        DeclareLaunchArgument('output_dir', default_value=os.path.join(
            os.path.expanduser('~'), 'FSR_robot', 'ros2_ws', 'test_bench_logs'),
            description='Directory where the telemetry CSV is written'),
        DeclareLaunchArgument('start_plotjuggler', default_value='true',
                              description='Also launch the PlotJuggler GUI alongside the simulation'),
        sim,
        logger,
        plotjuggler,
    ])
