"""
SeBaJu balance + vertical jump controller for ROS 2 Humble / Gazebo Fortress.

Port of SeBaJu_Ctrl_BalJumpAchieve.py (MuJoCo, position_servo leg mode):
  - PD position-and-velocity state feedback on the wheels (COM lean, lean rate, x, xdot)
  - seven-state hybrid jump supervisor with phase-dependent wheel gains
  - contact-force hysteresis + wheel clearance + vertical COM velocity for take-off/landing

MuJoCo -> ROS 2 mapping:
  data.qpos / qvel (joints)      -> /joint_states               (joint_state_broadcaster)
  torso freejoint pose           -> /sebaju/odom                (gz OdometryPublisher, ground truth)
  robot_com / wheel framepos     -> forward kinematics on the URDF (kinematics.py)
  mj_contactForce wheel/ground   -> /sebaju/{left,right}_wheel_contact (gz Contact sensor)
  wheel motor ctrl (gear 18)     -> /wheel_effort_controller/commands  [Nm]
  hip/knee position servo        -> /sebaju/<joint>/cmd_pos target angles [rad] (gz JointPositionController)
"""

import math

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Empty, Float64, Float64MultiArray, String

from sebaju_gazebo.kinematics import RobotKinematics, quat_to_matrix
from sebaju_gazebo.planar_trajectory import CubicSTrajectory

# =========================================================
# Geometry / actuation (SeBaJu_BOT(II).xml)
# =========================================================
WHEEL_RADIUS = 0.06
WHEEL_GEAR = 18.0
WHEEL_MOTOR_SIGN = -1.0
G = 9.81

LEG_JOINTS = ['left_hip', 'right_hip', 'left_knee', 'right_knee']
WHEEL_JOINTS = ['left_wheel_joint', 'right_wheel_joint']
LEFT_WHEEL_LINK = 'left_wheel_link'
RIGHT_WHEEL_LINK = 'right_wheel_link'

HIP_SERVO_KP = 160.0
KNEE_SERVO_KP = 200.0
HIP_SERVO_KV = 12.0
KNEE_SERVO_KV = 14.0
LEG_FORCE_LIMIT = 60.0
HIP_CTRL_MIN, HIP_CTRL_MAX = -0.60, 0.60
KNEE_CTRL_MIN, KNEE_CTRL_MAX = -0.80, 0.80

# Time the robot stays attached to the anchor with the legs held, before release.
HOLD_TIME = 1.0
RELEASE_REPEAT_TIME = 0.05

# =========================================================
# Balance controller
# =========================================================
PITCH_OFFSET = math.radians(4.73)
MAX_WHEEL = 0.35
K_THETA = 1.3
K_THETA_D = 0.17
K_X_D = 0.16
K_X = 0.8

COMZ_VEL_FILTER = 0.40

# =========================================================
# Planar trajectory tracking (differential wheel torque)
# =========================================================
PLANAR_START_TIME = 2.0
K_PSI = 0.10      # wheel_u per rad of heading error (wheel scrub friction needs ~5x the inertia-only value)
K_OMEGA = 0.025   # wheel_u per rad/s of yaw-rate error
K_LAT = 2.0       # rad of heading correction per m of lateral error (through atan)
MAX_YAW = 0.15
LAT_FULL_SPEED = 0.10  # m/s of reference speed at which lateral correction is fully active

# =========================================================
# Jump state machine
# =========================================================
STATE_BALANCE = 'BALANCE'
STATE_PRELOAD = 'PRELOAD'
STATE_THRUST = 'THRUST'
STATE_FLIGHT = 'FLIGHT'
STATE_LANDING = 'LANDING'
STATE_RECOVERY = 'RECOVERY'
STATE_SETTLE = 'SETTLE'
JUMP_ACTIVE_STATES = (STATE_PRELOAD, STATE_THRUST, STATE_FLIGHT, STATE_LANDING, STATE_RECOVERY)
POST_LANDING_STATES = (STATE_LANDING, STATE_RECOVERY, STATE_SETTLE)

JUMP_ONCE = True
T_PRELOAD = 0.80
T_THRUST = 0.02
T_THRUST_MAX = 0.16
T_LANDING = 0.80
T_RECOVERY = 2.00

F_CONTACT_OFF_RATIO = 0.03
F_CONTACT_ON_RATIO = 0.15
F_CONTACT_OFF_MIN = 1.5
F_CONTACT_ON_MIN = 6.0
CONTACT_LOSS_DEBOUNCE = 0.004
CONTACT_GAIN_DEBOUNCE = 0.012
# The gz contact sensor only publishes while touching; older messages mean "no contact".
CONTACT_STALE_TIME = 0.006
TAKEOFF_VZ_MIN = 0.08

WHEEL_CLEARANCE_TAKEOFF_ABS = 0.0005
WHEEL_CLEARANCE_TAKEOFF_REL = 0.0030

HIP_PRELOAD_DELTA = +0.12
KNEE_PRELOAD_DELTA = -0.22
HIP_THRUST_DELTA = -0.27
KNEE_THRUST_DELTA = +0.44
HIP_LAND_DELTA = +0.08
KNEE_LAND_DELTA = -0.20

SETTLE_LEAN_ERR = math.radians(2.0)
SETTLE_RATE = math.radians(8.0)
SETTLE_VEL = 0.04
SETTLE_WHEEL = 0.06
SETTLE_X_ERR = 0.015
SETTLE_HOLD = 0.30
MAX_SETTLE_WAIT = 12.0

PRINT_DT = {
    STATE_BALANCE: 0.50,
    STATE_PRELOAD: 0.40,
    STATE_THRUST: 0.01,
    STATE_FLIGHT: 0.04,
    STATE_LANDING: 0.08,
    STATE_RECOVERY: 0.20,
    STATE_SETTLE: 0.25,
}


def clamp(value, lo, hi):
    return max(lo, min(hi, float(value)))


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def smoothstep01(t, T):
    if T <= 0.0:
        return 1.0
    tau = clamp(t / T, 0.0, 1.0)
    return 3.0 * tau ** 2 - 2.0 * tau ** 3


def fast_thrust01(t, T):
    if T <= 0.0:
        return 1.0
    tau = clamp(t / T, 0.0, 1.0)
    return 1.0 - (1.0 - tau) ** 3


def blend(a, b, alpha):
    return a + (b - a) * alpha


def balance_gains_for_state(state):
    """(K_theta, K_theta_dot, K_x_dot, K_x, max_wheel) per jump phase."""
    if state == STATE_PRELOAD:
        return K_THETA, 0.22, 0.20, 0.50 * K_X, MAX_WHEEL
    if state == STATE_THRUST:
        return K_THETA, 0.25, 0.18, 0.10 * K_X, MAX_WHEEL
    if state == STATE_FLIGHT:
        return K_THETA, 0.10, 0.00, 0.00, 0.10
    if state == STATE_LANDING:
        return K_THETA, 0.18, 0.10, 0.00, 0.22
    if state == STATE_RECOVERY:
        return K_THETA, 0.20, 0.12, 0.20 * K_X, 0.28
    if state == STATE_SETTLE:
        return K_THETA, K_THETA_D, 0.22, 0.15 * K_X, MAX_WHEEL
    return K_THETA, K_THETA_D, K_X_D, K_X, MAX_WHEEL


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def stamp_to_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


SYNC_BUFFER_SIZE = 50


class BalanceJumpController(Node):

    def __init__(self):
        super().__init__('sebaju_balance_jump_controller')
        self.declare_parameter('robot_description', '')
        self.declare_parameter('jump_enable', True)
        self.declare_parameter('jump_start_time', 5.0)
        self.declare_parameter('return_to_start_after_jump', False)

        urdf_xml = self.get_parameter('robot_description').value
        if not urdf_xml:
            raise RuntimeError('Parameter robot_description is empty.')
        self.jump_enable = bool(self.get_parameter('jump_enable').value)
        self.jump_start_time = float(self.get_parameter('jump_start_time').value)
        self.return_to_start = bool(self.get_parameter('return_to_start_after_jump').value)
        self.declare_parameter('drive_enable', False)
        self.declare_parameter('drive_distance', 1.0)
        self.declare_parameter('drive_period', 8.0)
        self.declare_parameter('drive_start_time', 2.0)
        self.drive_enable = bool(self.get_parameter('drive_enable').value)
        self.drive_distance = float(self.get_parameter('drive_distance').value)
        self.drive_period = float(self.get_parameter('drive_period').value)
        self.drive_start_time = float(self.get_parameter('drive_start_time').value)
        self.declare_parameter('planar_enable', False)
        self.declare_parameter('traj_length', 3.0)
        self.declare_parameter('traj_lateral', 0.6)
        self.declare_parameter('traj_duration', 12.0)
        self.planar_enable = bool(self.get_parameter('planar_enable').value)
        self.trajectory = CubicSTrajectory(
            float(self.get_parameter('traj_length').value),
            float(self.get_parameter('traj_lateral').value),
            float(self.get_parameter('traj_duration').value)) if self.planar_enable else None

        self.kin = RobotKinematics(urdf_xml)
        self.robot_weight = self.kin.total_mass * G
        self.f_contact_off = max(F_CONTACT_OFF_MIN, F_CONTACT_OFF_RATIO * self.robot_weight)
        self.f_contact_on = max(F_CONTACT_ON_MIN, F_CONTACT_ON_RATIO * self.robot_weight)

        # Nominal CoM-to-axle offset of the XML standing pose (all joints 0, torso level).
        frames0 = self.kin.link_frames(np.zeros(3), np.eye(3), {})
        com0 = self.kin.center_of_mass(frames0)
        wheel_mid0 = 0.5 * (frames0[LEFT_WHEEL_LINK][1] + frames0[RIGHT_WHEEL_LINK][1])
        self.com_x_ref = float(com0[0] - wheel_mid0[0])

        self.get_logger().info(
            f'Robot mass = {self.kin.total_mass:.3f} kg, weight = {self.robot_weight:.3f} N, '
            f'contact OFF/ON = {self.f_contact_off:.3f}/{self.f_contact_on:.3f} N, '
            f'COM-axle x offset = {self.com_x_ref:.4f} m, '
            f'standing torso height = {WHEEL_RADIUS - wheel_mid0[2]:.4f} m')

        self.leg_pubs = [self.create_publisher(Float64, f'/sebaju/{name}/cmd_pos', 10) for name in LEG_JOINTS]
        self.wheel_pub = self.create_publisher(Float64MultiArray, '/wheel_effort_controller/commands', 10)
        self.release_pub = self.create_publisher(Empty, '/sebaju/release', 10)
        self.state_pub = self.create_publisher(String, '/sebaju/jump_state', 10)
        # [t, theta, theta_dot, x, xdot, wheel_u, com_z, com_z_vel, Fn_total, min_wheel_gap, loaded]
        self.debug_pub = self.create_publisher(Float64MultiArray, '/sebaju/debug', 10)
        # [t, x_ref, y_ref, heading_ref, x, y, heading, along_err, lateral_err, yaw_u] (world frame, axle midpoint)
        self.planar_pub = self.create_publisher(Float64MultiArray, '/sebaju/planar', 10)
        # [err_x, err_y]: actual - desired position, world frame x/y (axle midpoint), while following the planar trajectory
        self.tracking_error_pub = self.create_publisher(Float64MultiArray, '/sebaju/tracking_error', 10)

        self.odom = None
        self.latest_odom = None
        self.latest_joint_state = None
        self.joint_state_buffer = {}
        self.odom_buffer = {}
        self.warned_no_sync = False
        self.sync_count = 0
        self.imu = None
        self.contact = {'left': (-math.inf, 0.0), 'right': (-math.inf, 0.0)}
        self.warned_no_wrench = False

        self.create_subscription(Odometry, '/sebaju/odom', self._odom_cb, 10)
        self.create_subscription(Imu, '/sebaju/imu', self._imu_cb, 10)
        self.create_subscription(Contacts, '/sebaju/left_wheel_contact',
                                 lambda msg: self._contact_cb(msg, 'left'), 10)
        self.create_subscription(Contacts, '/sebaju/right_wheel_contact',
                                 lambda msg: self._contact_cb(msg, 'right'), 10)
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)

        self.phase = 'HOLD'
        self.hold_start = None
        self.t_release = None
        self.last_stamp = None
        self._init_control_state()
        self.get_logger().info('Waiting for /joint_states and /sebaju/odom ...')

    # -----------------------------------------------------------------
    # State
    # -----------------------------------------------------------------
    def _init_control_state(self):
        self.jump_state = STATE_BALANCE
        self.jump_done = False
        self.loaded_state = True
        self.state_t0 = 0.0

        self.prev_com_z = None
        self.com_z_vel_filtered = 0.0
        self.prev_theta = None
        self.theta_dot = 0.0
        self.xdot = 0.0
        self.prev_x = 0.0
        self.axle_origin = None
        self.axle_heading0 = None
        self.yaw0 = 0.0
        self.prev_yaw = 0.0
        self.prev_axle_xy = None

        self.unloaded_timer = 0.0
        self.loaded_timer = 0.0

        self.x_ref_active = 0.0
        self.last_print = -math.inf

        self.hip_flight_hold = (0.0, 0.0)
        self.knee_flight_hold = (0.0, 0.0)
        self.flight_hold_valid = False
        self.landing_start_hip = (0.0, 0.0)
        self.landing_start_knee = (0.0, 0.0)
        self.thrust_start_time = None

        self._reset_jump_metrics(0.0, 0.0)

    def _reset_jump_metrics(self, com_z, min_wheel_gap):
        self.stand_com_z = com_z
        self.preload_min_com_z = com_z
        self.wheel_gap_ref = None
        self.peak_gap_abs = min_wheel_gap
        self.peak_gap_rel = 0.0
        self.peak_com_z = com_z
        self.peak_air_com_z = -math.inf
        self.takeoff_time = None
        self.takeoff_com_z = None
        self.takeoff_com_z_vel = None
        self.landing_time = None
        self.flight_time = 0.0
        self.jump_height_airborne = 0.0
        self.jump_height_above_stand_air = 0.0
        self.v_takeoff_from_air = 0.0
        self.thrust_force_sum = 0.0
        self.thrust_force_count = 0
        self.average_thrust_force = 0.0
        self.settle_timer = 0.0
        self.settle_time = None
        self.wheel_sat_count = 0
        self.leg_sat_count = 0
        self.max_abs_wheel_u = 0.0
        self.max_abs_xdot_after_landing = 0.0
        self.max_abs_theta_dot_after_landing = 0.0

    def _set_state(self, state, t):
        self.jump_state = state
        self.state_t0 = t
        self.state_pub.publish(String(data=state))

    # -----------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------
    def _odom_cb(self, msg):
        key = stamp_to_ns(msg.header.stamp)
        self.latest_odom = msg
        self.odom_buffer[key] = msg
        self._try_control(key, from_joint_state=False)

    def _imu_cb(self, msg):
        self.imu = msg

    def _contact_cb(self, msg, side):
        normal_force = 0.0
        ground_contact = False
        has_wrench = False
        for contact in msg.contacts:
            if 'ground' not in contact.collision1.name and 'ground' not in contact.collision2.name:
                continue
            ground_contact = True
            for wrench in contact.wrenches:
                has_wrench = True
                f = wrench.body_1_wrench.force
                normal_force += math.sqrt(f.x * f.x + f.y * f.y + f.z * f.z)
        if not ground_contact:
            return
        if not has_wrench:
            # Physics engine did not report forces: treat a touching wheel as carrying half the weight.
            normal_force = 0.5 * self.robot_weight
            if not self.warned_no_wrench:
                self.warned_no_wrench = True
                self.get_logger().warn('Contact messages carry no wrench; using weight-based contact force.')
        stamp = stamp_to_sec(msg.header.stamp)
        if stamp <= 0.0:
            stamp = self.get_clock().now().nanoseconds * 1e-9
        self.contact[side] = (stamp, normal_force)

    def _joint_state_cb(self, msg):
        self.latest_joint_state = msg
        key = stamp_to_ns(msg.header.stamp)
        self.joint_state_buffer[key] = msg
        self._try_control(key, from_joint_state=True)

    def _try_control(self, key, from_joint_state):
        # Joint states and odometry arrive through different transports: only pair samples of the same
        # sim step, otherwise the torso pose lags the joints by a step and the COM lean rate gets noisy.
        for buffer in (self.joint_state_buffer, self.odom_buffer):
            if len(buffer) > SYNC_BUFFER_SIZE:
                del buffer[min(buffer)]
                if not self.warned_no_sync:
                    self.warned_no_sync = True
                    self.get_logger().warn(
                        '/joint_states and /sebaju/odom stamps do not coincide: the odometry publish '
                        'frequency must equal the controller_manager update_rate. '
                        f'JS buffer: {len(self.joint_state_buffer)}, Odom buffer: {len(self.odom_buffer)}')
        if key not in self.joint_state_buffer or key not in self.odom_buffer:
            # Fallback: stamps never coincide. Step only on the ground-truth odometry (500 Hz) so each pose is
            # processed once with its exact stamp, paired with the newest joint state.
            if from_joint_state:
                return
            if self.latest_joint_state is not None and self.latest_odom is not None:
                self.sync_count += 1
                if self.sync_count == 1:
                    self.get_logger().warn('Timestamp sync failed, falling back to latest-of-each')
                msg = self.latest_joint_state
                self.odom = self.latest_odom
            else:
                return
        else:
            # Perfect sync
            msg = self.joint_state_buffer.pop(key)
            self.odom = self.odom_buffer.pop(key)
            for buffer in (self.joint_state_buffer, self.odom_buffer):
                for old_key in [k for k in buffer if k < key]:
                    del buffer[old_key]

        t_abs = key * 1e-9
        if self.last_stamp is None:
            self.last_stamp = t_abs
            return
        dt = t_abs - self.last_stamp
        if dt <= 0.0:
            return
        self.last_stamp = t_abs

        q = dict(zip(msg.name, msg.position))
        qd = dict(zip(msg.name, msg.velocity))
        if any(name not in q for name in LEG_JOINTS + WHEEL_JOINTS):
            return

        if self.phase == 'HOLD':
            self._hold_step(t_abs, q, qd)
        else:
            self._control_step(t_abs, dt, q, qd)

    # -----------------------------------------------------------------
    # Hold / release
    # -----------------------------------------------------------------
    def _publish_legs(self, targets, q, qd):
        """Send hip/knee target angles; returns the estimated servo force for saturation diagnostics."""
        kps = (HIP_SERVO_KP, HIP_SERVO_KP, KNEE_SERVO_KP, KNEE_SERVO_KP)
        kvs = (HIP_SERVO_KV, HIP_SERVO_KV, KNEE_SERVO_KV, KNEE_SERVO_KV)
        servo_forces = []
        for pub, kp, kv, target, name in zip(self.leg_pubs, kps, kvs, targets, LEG_JOINTS):
            pub.publish(Float64(data=float(target)))
            servo_forces.append(kp * (target - q[name]) - kv * qd[name])
        return servo_forces

    def _publish_wheels(self, wheel_u, yaw_u=0.0):
        left = WHEEL_GEAR * clamp(wheel_u - yaw_u, -1.0, 1.0)
        right = WHEEL_GEAR * clamp(wheel_u + yaw_u, -1.0, 1.0)
        self.wheel_pub.publish(Float64MultiArray(data=[left, right]))

    def _hold_step(self, t_abs, q, qd):
        if self.hold_start is None:
            self.hold_start = t_abs
            self.get_logger().info('Holding legs on the anchor ...')
        self._publish_legs((0.0, 0.0, 0.0, 0.0), q, qd)
        self._publish_wheels(0.0)
        if t_abs - self.hold_start < HOLD_TIME:
            return

        self.release_pub.publish(Empty())
        self.phase = 'RUN'
        self.t_release = t_abs
        self._set_state(STATE_BALANCE, 0.0)
        self.get_logger().info(f'Released at sim t={t_abs:.3f}s. Balance controller active.')

    def _drive_reference(self, t):
        """Back-and-forth along the initial heading: 0 -> drive_distance -> 0, zero speed at both ends."""
        s = t - self.drive_start_time
        if s <= 0.0:
            return 0.0, 0.0
        w = 2.0 * math.pi / self.drive_period
        return (0.5 * self.drive_distance * (1.0 - math.cos(w * s)),
                0.5 * self.drive_distance * w * math.sin(w * s))

    def _planar_tracking(self, t, axle_xy, yaw, forward_speed, yaw_rate):
        """Track the planar trajectory: returns (along-track pos error, speed error, differential wheel_u)."""
        s = t - PLANAR_START_TIME
        px, py, heading_ref, v_ref, yaw_rate_ref = self.trajectory.sample(max(s, 0.0))
        if s <= 0.0:
            v_ref = yaw_rate_ref = 0.0
        c0, s0 = math.cos(self.yaw0), math.sin(self.yaw0)
        p_ref = self.axle_origin[:2] + np.array([c0 * px - s0 * py, s0 * px + c0 * py])
        yaw_ref = self.yaw0 + heading_ref

        err = axle_xy - p_ref
        tangent = np.array([math.cos(yaw_ref), math.sin(yaw_ref)])
        normal = np.array([-math.sin(yaw_ref), math.cos(yaw_ref)])
        along_err = float(np.dot(err, tangent))
        lateral_err = float(np.dot(err, normal))

        # A differential-drive robot can only cancel lateral error while moving: fade the correction out near standstill.
        yaw_cmd = yaw_ref - math.atan(K_LAT * lateral_err) * min(1.0, abs(v_ref) / LAT_FULL_SPEED)
        yaw_u = clamp(K_PSI * wrap_angle(yaw_cmd - yaw) + K_OMEGA * (yaw_rate_ref - yaw_rate), -MAX_YAW, MAX_YAW)

        self.planar_pub.publish(Float64MultiArray(data=[
            t, float(p_ref[0]), float(p_ref[1]), yaw_ref, float(axle_xy[0]), float(axle_xy[1]), yaw,
            along_err, lateral_err, yaw_u]))
        self.tracking_error_pub.publish(Float64MultiArray(data=[float(err[0]), float(err[1])]))
        return along_err, forward_speed - v_ref, yaw_u

    def _base_pose(self):
        p = self.odom.pose.pose.position
        o = self.odom.pose.pose.orientation
        return np.array([p.x, p.y, p.z]), quat_to_matrix(o.x, o.y, o.z, o.w)

    def _contact_force(self, side, t_abs):
        stamp, force = self.contact[side]
        return force if t_abs - stamp <= CONTACT_STALE_TIME else 0.0

    # -----------------------------------------------------------------
    # Main control step (one MuJoCo loop iteration)
    # -----------------------------------------------------------------
    def _control_step(self, t_abs, dt, q, qd):
        t = t_abs - self.t_release
        if t < RELEASE_REPEAT_TIME:
            self.release_pub.publish(Empty())

        base_pos, base_rot = self._base_pose()
        frames = self.kin.link_frames(base_pos, base_rot, q)
        com = self.kin.center_of_mass(frames)
        wl = frames[LEFT_WHEEL_LINK][1]
        wr = frames[RIGHT_WHEEL_LINK][1]

        # ---------------- vertical CoM velocity ----------------
        com_z = float(com[2])
        if self.prev_com_z is None:
            self.prev_com_z = com_z
            self._reset_jump_metrics(com_z, min(wl[2], wr[2]) - WHEEL_RADIUS)
        com_z_vel_raw = (com_z - self.prev_com_z) / dt
        self.prev_com_z = com_z
        self.com_z_vel_filtered = COMZ_VEL_FILTER * self.com_z_vel_filtered + (1.0 - COMZ_VEL_FILTER) * com_z_vel_raw

        if self.jump_state == STATE_PRELOAD:
            self.preload_min_com_z = min(self.preload_min_com_z, com_z)

        # ---------------- wheel clearance ----------------
        min_wheel_gap = min(float(wl[2]), float(wr[2])) - WHEEL_RADIUS
        gap_rel = 0.0 if self.wheel_gap_ref is None else min_wheel_gap - self.wheel_gap_ref
        if self.jump_state in JUMP_ACTIVE_STATES:
            self.peak_gap_abs = max(self.peak_gap_abs, min_wheel_gap)
            self.peak_gap_rel = max(self.peak_gap_rel, gap_rel)
        wheel_clear_air = (min_wheel_gap > WHEEL_CLEARANCE_TAKEOFF_ABS
                           and gap_rel > WHEEL_CLEARANCE_TAKEOFF_REL)

        # ---------------- COM lean relative to the wheel axle ----------------
        wheel_mid = 0.5 * (wl + wr)
        heading = base_rot[:, 0].copy()
        heading[2] = 0.0
        heading /= max(np.linalg.norm(heading), 1e-9)
        dx = float(np.dot(com - wheel_mid, heading)) - self.com_x_ref
        dz = float(com[2] - wheel_mid[2])
        theta = -math.atan2(dx, max(dz, 1e-6))
        if self.prev_theta is None:
            self.prev_theta = theta
        # Ideal ground-truth state: exact derivatives on simulator stamps, no filtering.
        theta_dot = (theta - self.prev_theta) / dt
        self.prev_theta = theta
        self.theta_dot = theta_dot

        # ---------------- axle position (ground truth, along the heading at release) ----------------
        yaw = math.atan2(heading[1], heading[0])
        if self.axle_origin is None:
            self.axle_origin = wheel_mid.copy()
            self.axle_heading0 = heading.copy()
            self.yaw0 = yaw
            self.prev_x = 0.0
            self.prev_yaw = yaw
            self.prev_axle_xy = wheel_mid[:2].copy()
        x = float(np.dot(wheel_mid - self.axle_origin, self.axle_heading0))
        xdot = (x - self.prev_x) / dt
        self.prev_x = x
        self.xdot = xdot
        axle_xy = wheel_mid[:2].copy()
        forward_speed = float(np.dot((axle_xy - self.prev_axle_xy) / dt, heading[:2]))
        self.prev_axle_xy = axle_xy
        yaw_rate = wrap_angle(yaw - self.prev_yaw) / dt
        self.prev_yaw = yaw

        # ---------------- contact force hysteresis ----------------
        fn_total = self._contact_force('left', t_abs) + self._contact_force('right', t_abs)
        if self.jump_state == STATE_THRUST:
            self.thrust_force_sum += fn_total
            self.thrust_force_count += 1

        if self.loaded_state:
            self.unloaded_timer = self.unloaded_timer + dt if fn_total < self.f_contact_off else 0.0
            if self.unloaded_timer >= CONTACT_LOSS_DEBOUNCE:
                self.loaded_state = False
                self.loaded_timer = 0.0
        else:
            self.loaded_timer = self.loaded_timer + dt if fn_total > self.f_contact_on else 0.0
            if self.loaded_timer >= CONTACT_GAIN_DEBOUNCE:
                self.loaded_state = True
                self.unloaded_timer = 0.0
        airborne = not self.loaded_state

        if self.jump_state in JUMP_ACTIVE_STATES:
            self.peak_com_z = max(self.peak_com_z, com_z)
        if self.jump_state == STATE_FLIGHT:
            self.peak_air_com_z = max(self.peak_air_com_z, com_z)

        # ---------------- stage-aware wheel balance ----------------
        theta_ref = PITCH_OFFSET
        xdot_ref = 0.0
        if self.jump_state in POST_LANDING_STATES or (self.jump_state == STATE_BALANCE and self.jump_done):
            x_ref = self.x_ref_active
        else:
            x_ref = 0.0
        if self.drive_enable and self.jump_state == STATE_BALANCE and not self.jump_done:
            x_ref, xdot_ref = self._drive_reference(t)
        pos_err = x - x_ref
        vel_err = xdot - xdot_ref

        yaw_u = 0.0
        if self.planar_enable and self.jump_state == STATE_BALANCE and not self.jump_done:
            pos_err, vel_err, yaw_u = self._planar_tracking(
                t, axle_xy, yaw, forward_speed, yaw_rate)

        k_th, k_thd, k_xd, k_x, max_wheel_state = balance_gains_for_state(self.jump_state)
        wheel_u_raw = clamp(
            k_th * (theta - theta_ref) + k_thd * theta_dot - k_xd * vel_err - k_x * pos_err,
            -max_wheel_state, max_wheel_state)
        wheel_u = WHEEL_MOTOR_SIGN * wheel_u_raw
        wheel_sat = abs(wheel_u) >= 0.98 * max_wheel_state

        # ---------------- settling check ----------------
        if self.landing_time is not None and self.jump_state in (STATE_RECOVERY, STATE_SETTLE, STATE_BALANCE):
            settled_now = (self.loaded_state
                           and abs(theta - PITCH_OFFSET) < SETTLE_LEAN_ERR
                           and abs(theta_dot) < SETTLE_RATE
                           and abs(xdot) < SETTLE_VEL
                           and abs(wheel_u) < SETTLE_WHEEL
                           and abs(x - self.x_ref_active) < SETTLE_X_ERR)
            self.settle_timer = self.settle_timer + dt if settled_now else 0.0
        else:
            self.settle_timer = 0.0

        # ---------------- jump state machine ----------------
        stable_for_jump = (abs(theta - theta_ref) < math.radians(2.5)
                           and abs(xdot) < 0.08
                           and self.loaded_state)
        elapsed = t - self.state_t0

        if self.jump_state == STATE_BALANCE:
            if self.jump_enable and not self.jump_done and t > self.jump_start_time and stable_for_jump:
                self._set_state(STATE_PRELOAD, t)
                self._reset_jump_metrics(com_z, min_wheel_gap)
                self.wheel_gap_ref = min_wheel_gap
                self.get_logger().info(f'>>> JUMP STATE: PRELOAD t={t:.4f}s StandZ={com_z:.4f}')

        elif self.jump_state == STATE_PRELOAD:
            if elapsed >= T_PRELOAD:
                self._set_state(STATE_THRUST, t)
                self.thrust_start_time = t
                self.thrust_force_sum = 0.0
                self.thrust_force_count = 0
                self.get_logger().info(
                    f'>>> JUMP STATE: THRUST t={t:.4f}s COMz={com_z:.4f} PreloadMinZ={self.preload_min_com_z:.4f}')

        elif self.jump_state == STATE_THRUST:
            takeoff_detected = (airborne
                                and fn_total < self.f_contact_off
                                and wheel_clear_air
                                and com_z_vel_raw > TAKEOFF_VZ_MIN)
            if takeoff_detected:
                self._set_state(STATE_FLIGHT, t)
                self.takeoff_time = t
                self.takeoff_com_z = com_z
                self.takeoff_com_z_vel = com_z_vel_raw
                self.peak_air_com_z = com_z
                self.hip_flight_hold = (q['left_hip'], q['right_hip'])
                self.knee_flight_hold = (q['left_knee'], q['right_knee'])
                self.flight_hold_valid = True
                if self.thrust_force_count > 0:
                    self.average_thrust_force = self.thrust_force_sum / self.thrust_force_count
                self.get_logger().info(
                    f'>>> TAKE-OFF DETECTED t={t:.4f}s COMz={com_z:.4f} Vz_raw={com_z_vel_raw:.4f} '
                    f'Fn={fn_total:.2f}N GapAbs={min_wheel_gap:.4f}m GapRel={gap_rel:.4f}m')
            elif elapsed >= T_THRUST_MAX:
                self._set_state(STATE_LANDING, t)
                self.get_logger().info(
                    f'>>> NO TAKE-OFF: SAFE LANDING t={t:.4f}s COMz={com_z:.4f} Fn={fn_total:.2f}N')

        elif self.jump_state == STATE_FLIGHT:
            self.peak_air_com_z = max(self.peak_air_com_z, com_z)
            if self.loaded_state:
                self.landing_start_hip = (q['left_hip'], q['right_hip'])
                self.landing_start_knee = (q['left_knee'], q['right_knee'])
                self._set_state(STATE_LANDING, t)
                self.landing_time = t
                self.x_ref_active = 0.0 if self.return_to_start else x
                self.settle_timer = 0.0
                self.settle_time = None
                self.flight_time = t - self.takeoff_time
                self.jump_height_airborne = max(0.0, self.peak_air_com_z - self.takeoff_com_z)
                self.jump_height_above_stand_air = max(0.0, self.peak_air_com_z - self.stand_com_z)
                self.v_takeoff_from_air = math.sqrt(2.0 * G * self.jump_height_airborne)
                self.get_logger().info(
                    f'>>> LANDING DETECTED t={t:.4f}s FlightTime={self.flight_time:.4f}s '
                    f'TakeoffZ={self.takeoff_com_z:.4f} PeakAirZ={self.peak_air_com_z:.4f} '
                    f'LandingZ={com_z:.4f} H_airborne={self.jump_height_airborne:.4f}m '
                    f'Vto_air_est={self.v_takeoff_from_air:.4f}m/s Favg_thrust={self.average_thrust_force:.2f}N')

        elif self.jump_state == STATE_LANDING:
            if elapsed >= T_LANDING:
                self._set_state(STATE_RECOVERY, t)
                self.get_logger().info('>>> JUMP STATE: RECOVERY')

        elif self.jump_state == STATE_RECOVERY:
            if elapsed >= T_RECOVERY:
                self._set_state(STATE_SETTLE, t)
                self.get_logger().info('>>> JUMP STATE: SETTLE / WAITING FOR STABLE BALANCE')

        elif self.jump_state == STATE_SETTLE:
            if self.settle_time is None and self.settle_timer >= SETTLE_HOLD:
                self.settle_time = t - self.landing_time
            settle_timeout = self.landing_time is not None and (t - self.landing_time) >= MAX_SETTLE_WAIT
            if self.settle_time is not None or settle_timeout:
                self._finish_jump(t)

        # ---------------- hip/knee targets ----------------
        elapsed = t - self.state_t0
        stand = (0.0, 0.0, 0.0, 0.0)  # hip_L, hip_R, knee_L, knee_R of the XML standing pose
        preload = (HIP_PRELOAD_DELTA, HIP_PRELOAD_DELTA, KNEE_PRELOAD_DELTA, KNEE_PRELOAD_DELTA)
        thrust = (HIP_THRUST_DELTA, HIP_THRUST_DELTA, KNEE_THRUST_DELTA, KNEE_THRUST_DELTA)
        land = (HIP_LAND_DELTA, HIP_LAND_DELTA, KNEE_LAND_DELTA, KNEE_LAND_DELTA)

        if self.jump_state == STATE_PRELOAD:
            a = smoothstep01(elapsed, T_PRELOAD)
            targets = [blend(s, p, a) for s, p in zip(stand, preload)]
        elif self.jump_state == STATE_THRUST:
            a = fast_thrust01(elapsed, T_THRUST)
            targets = [blend(p, th, a) for p, th in zip(preload, thrust)]
        elif self.jump_state == STATE_FLIGHT:
            if self.flight_hold_valid:
                targets = [*self.hip_flight_hold, *self.knee_flight_hold]
            else:
                targets = list(thrust)
        elif self.jump_state == STATE_LANDING:
            a = smoothstep01(elapsed, T_LANDING)
            start = (*self.landing_start_hip, *self.landing_start_knee)
            targets = [blend(s, ld, a) for s, ld in zip(start, land)]
        elif self.jump_state == STATE_RECOVERY:
            a = smoothstep01(elapsed, T_RECOVERY)
            targets = [blend(ld, s, a) for ld, s in zip(land, stand)]
        else:
            targets = list(stand)

        targets = [clamp(targets[0], HIP_CTRL_MIN, HIP_CTRL_MAX),
                   clamp(targets[1], HIP_CTRL_MIN, HIP_CTRL_MAX),
                   clamp(targets[2], KNEE_CTRL_MIN, KNEE_CTRL_MAX),
                   clamp(targets[3], KNEE_CTRL_MIN, KNEE_CTRL_MAX)]

        leg_torques = self._publish_legs(targets, q, qd)
        self._publish_wheels(wheel_u, yaw_u)
        leg_sat = any(abs(tau) > 0.98 * LEG_FORCE_LIMIT for tau in leg_torques)

        if self.jump_state in POST_LANDING_STATES:
            self.wheel_sat_count += int(wheel_sat)
            self.leg_sat_count += int(leg_sat)
            self.max_abs_wheel_u = max(self.max_abs_wheel_u, abs(wheel_u))
            self.max_abs_xdot_after_landing = max(self.max_abs_xdot_after_landing, abs(xdot))
            self.max_abs_theta_dot_after_landing = max(self.max_abs_theta_dot_after_landing, abs(theta_dot))

        self.debug_pub.publish(Float64MultiArray(data=[
            t, theta, theta_dot, x, xdot, wheel_u, com_z, com_z_vel_raw, fn_total, min_wheel_gap,
            float(self.loaded_state)]))

        if t - self.last_print >= PRINT_DT.get(self.jump_state, 0.2):
            self.last_print = t
            if wheel_sat:
                status = 'WHEEL_SAT'
            elif leg_sat:
                status = 'LEG_SAT'
            elif abs(theta - theta_ref) < 0.035 and abs(xdot) < 0.08:
                status = 'BALANCING'
            elif abs(xdot) > 1.0:
                status = 'FAST MOTION'
            else:
                status = 'correcting'
            self.get_logger().info(
                f't={t:6.2f} {self.jump_state:>8} CoMLean={math.degrees(theta):+6.2f}deg '
                f'Rate={math.degrees(theta_dot):+7.1f}deg/s x={x:+6.3f} v={xdot:+6.3f} '
                f'WheelU={wheel_u:+6.3f} COMz={com_z:.4f} Fn={fn_total:6.2f}N '
                f'Gap={min_wheel_gap:+.4f}m loaded={int(self.loaded_state)} '
                f'Hip={targets[0]:+.2f} Knee={targets[2]:+.2f} {status}')

    def _finish_jump(self, t):
        settle_failed = self.settle_time is None
        jump_height_total = max(0.0, self.peak_com_z - self.stand_com_z)
        jump_height_from_crouch = max(0.0, self.peak_com_z - self.preload_min_com_z)
        visible_jump = self.peak_gap_abs >= 0.010 and self.jump_height_above_stand_air >= 0.010

        self._set_state(STATE_BALANCE, t)
        self.jump_done = JUMP_ONCE
        self.get_logger().info('>>> JUMP STATE: BALANCE / SETTLED')
        self.get_logger().info(
            f'>>> FINAL JUMP SUMMARY FlightTime={self.flight_time:.4f}s '
            f'H_airborne={self.jump_height_airborne:.4f}m H_total={jump_height_total:.4f}m '
            f'H_from_crouch={jump_height_from_crouch:.4f}m PeakGapAbs={self.peak_gap_abs:.4f}m '
            f'PeakGapRel={self.peak_gap_rel:.4f}m Vto_air_est={self.v_takeoff_from_air:.4f}m/s '
            f'Favg_thrust={self.average_thrust_force:.2f}N '
            f'SettleTime={-1.0 if settle_failed else self.settle_time:.4f} TimedOut={int(settle_failed)} '
            f'WheelSatCount={self.wheel_sat_count} LegSatCount={self.leg_sat_count} '
            f'MaxWheelU={self.max_abs_wheel_u:.3f} MaxPostLandVel={self.max_abs_xdot_after_landing:.3f} '
            f'MaxPostLandRate={math.degrees(self.max_abs_theta_dot_after_landing):.1f}deg/s '
            f'SUCCESS_VISIBLE={int(visible_jump)}')


def main(args=None):
    rclpy.init(args=args)
    node = BalanceJumpController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
