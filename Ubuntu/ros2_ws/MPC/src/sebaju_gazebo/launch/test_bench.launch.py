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

    output_dir = LaunchConfiguration('output_dir')
    start_plotjuggler = LaunchConfiguration('start_plotjuggler')

    # Controller arguments (jump_enable, height_enable, ...) are inherited by the included launch file.
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'sebaju_gazebo.launch.py')))

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
        # <workspace>/install/sebaju_gazebo/share/sebaju_gazebo -> <workspace>/test_bench_logs
        DeclareLaunchArgument('output_dir', default_value=os.path.join(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(pkg_share)))),
            'test_bench_logs'),
            description='Directory where the telemetry CSV is written'),
        DeclareLaunchArgument('start_plotjuggler', default_value='true',
                              description='Also launch the PlotJuggler GUI alongside the simulation'),
        sim,
        logger,
        plotjuggler,
    ])
