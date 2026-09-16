import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

SPAWN_HEIGHT = 0.2439  # torso height of SeBaJu_BOT(II).xml
# The Fortress contact system ignores <topic> and publishes on the scoped sensor topic.
LEFT_CONTACT_GZ_TOPIC = '/world/sebaju_world/model/sebaju/link/left_wheel_link/sensor/left_wheel_contact/contact'
RIGHT_CONTACT_GZ_TOPIC = '/world/sebaju_world/model/sebaju/link/right_wheel_link/sensor/right_wheel_contact/contact'

# (name, default, type, description) forwarded as parameters to wbr_controller
CONTROLLER_ARGS = [
    ('mpc_enable', 'true', bool, 'Upper-body MPC + stance VMC; false = rigid legs, TV-LQR only'),
    ('jump_enable', 'false', bool, 'Jump (squat, thrust, flight, landing) at jump_start_time'),
    ('jump_start_time', '4.0', float, 'Seconds after release before the jump starts'),
    ('jump_velocity', '1.2', float, 'CoM vertical velocity at the end of the thrust [m/s]'),
    ('height_enable', 'false', bool, 'Sinusoidal standing-height change (paper Sec. 5.1.1)'),
    ('height_amplitude', '0.03', float, 'Height oscillation amplitude [m]'),
    ('height_period', '2.2', float, 'Height oscillation period [s]'),
    ('velocity_enable', 'false', bool, 'Trapezoidal velocity profile (paper Sec. 5.1.3)'),
    ('velocity_max', '0.44', float, 'Cruise velocity [m/s]'),
    ('accel_max', '0.6', float, 'Acceleration / deceleration [m/s^2]'),
    ('velocity_distance', '2.0', float, 'Total travelled distance [m]'),
    ('drive_enable', 'false', bool, 'Back-and-forth reference while balancing'),
    ('drive_distance', '1.0', float, 'Back-and-forth travel [m]'),
    ('drive_period', '8.0', float, 'Duration of one back-and-forth cycle [s]'),
    ('planar_enable', 'false', bool, 'Follow the planar polynomial S-trajectory'),
    ('traj_length', '3.0', float, 'Trajectory arc length [m]'),
    ('traj_lateral', '0.6', float, 'Lateral offset of the S-curve [m]'),
    ('traj_duration', '12.0', float, 'Trajectory duration [s]'),
    ('push_enable', 'false', bool, 'Horizontal impulse on the torso (paper Sec. 5.1.2)'),
    ('push_time', '4.0', float, 'Seconds after release when the impulse is applied'),
    ('push_impulse', '2.7', float, 'Impulse [N s] (paper 50 kg x 1.8 m/s rescaled)'),
]


def generate_launch_description():
    pkg_share = get_package_share_directory('sebaju_gazebo')
    world_file = os.path.join(pkg_share, 'worlds', 'sebaju_world.sdf')
    xacro_file = os.path.join(pkg_share, 'urdf', 'sebaju.urdf.xacro')
    controllers_file = os.path.join(pkg_share, 'config', 'sebaju_controllers.yaml')

    gui = LaunchConfiguration('gui')

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
            '/world/sebaju_world/wrench@ros_gz_interfaces/msg/EntityWrench]gz.msgs.EntityWrench',
        ],
        remappings=[
            (LEFT_CONTACT_GZ_TOPIC, '/sebaju/left_wheel_contact'),
            (RIGHT_CONTACT_GZ_TOPIC, '/sebaju/right_wheel_contact'),
        ],
    )

    def spawner(name):
        return Node(package='controller_manager', executable='spawner',
                    arguments=[name, '--controller-manager', '/controller_manager'])

    joint_state_broadcaster_spawner = spawner('joint_state_broadcaster')
    wheel_controller_spawner = spawner('wheel_effort_controller')
    leg_controller_spawner = spawner('leg_effort_controller')

    wbr_controller = Node(
        package='sebaju_gazebo',
        executable='wbr_controller',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('start_controller')),
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
            **{name: ParameterValue(LaunchConfiguration(name), value_type=typ)
               for name, _, typ, _ in CONTROLLER_ARGS},
        }],
    )

    dashboard = ExecuteProcess(
        cmd=['ros2', 'run', 'sebaju_dashboard', 'dashboard'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('dashboard')),
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Start the Gazebo GUI'),
        DeclareLaunchArgument('dashboard', default_value='false',
                              description='Open the real-time sebaju_dashboard GUI'),
        DeclareLaunchArgument('start_controller', default_value='true',
                              description='Start wbr_controller here; set false to launch it by hand'),
        *[DeclareLaunchArgument(name, default_value=default, description=desc)
          for name, default, _, desc in CONTROLLER_ARGS],
        gz_sim,
        bridge,
        dashboard,
        robot_state_publisher,
        spawn_robot,
        RegisterEventHandler(OnProcessExit(target_action=spawn_robot,
                                           on_exit=[joint_state_broadcaster_spawner])),
        RegisterEventHandler(OnProcessExit(target_action=joint_state_broadcaster_spawner,
                                           on_exit=[wheel_controller_spawner])),
        RegisterEventHandler(OnProcessExit(target_action=wheel_controller_spawner,
                                           on_exit=[leg_controller_spawner])),
        RegisterEventHandler(OnProcessExit(target_action=leg_controller_spawner,
                                           on_exit=[wbr_controller])),
    ])
