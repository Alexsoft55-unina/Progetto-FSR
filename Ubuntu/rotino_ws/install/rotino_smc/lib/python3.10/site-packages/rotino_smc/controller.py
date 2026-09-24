"""
RoTino wheeled-biped controller: pure sliding-mode cascade on the model of Cui et al., "Modeling and
Control of a Wheeled Biped Robot", Micromachines 2022, 13, 747.

  state estimator (Sec. 4.3)   IMU + wheel odometry + leg kinematics -> linear Kalman filter, 500 Hz
  outer loop                   position -> velocity -> pitch reference theta*, PI with NEGATIVE gains
  pitch super-twisting         s1 = theta_dot + c1 (theta - theta*) -> common wheel torque, 500 Hz
  yaw PD                       heading error -> differential wheel torque, 500 Hz
  per-leg task-space SMC       z_ref, F_z -> hip/knee torques, 500 Hz

The balance law stays unified on the common wheel torque: the torso pitch is a single shared degree of
freedom, so two independent per-leg controllers would fight over the same sliding surface. The legs, by
contrast, are two separate planar 2-link chains: each closes the loop on its own kinematics while
sharing the reference, which is the symmetry hypothesis of Sec. 3 of the paper.

Only proprioceptive/IMU data drive the controller; /rotino/odom (ground truth) is used only to publish
the estimation error.
"""

import math
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts, Entity, EntityWrench
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Empty, Float64, Float64MultiArray, String

from rotino_description.kinematics import quat_to_matrix
from rotino_description.planar_trajectory import CubicSTrajectory
from rotino_description.model import G, WBRModel
from rotino_smc.estimator import AxisKalman

LEG_JOINTS = ['left_hip', 'right_hip', 'left_knee', 'right_knee']
WHEEL_JOINTS = ['left_wheel_joint', 'right_wheel_joint']

HOLD_TIME = 1.5
RELEASE_REPEAT_TIME = 0.05
MOTION_START_TIME = 2.0      # reference motions start this long after release
PHYSICS_DT = 0.0005          # rotino_world.sdf max_step_size (push impulse)
PUSH_TICKS = 25

# ---------------- outer loop: position -> velocity -> pitch reference ----------------
# Non-minimum phase: to accelerate forward the robot must first lean forward, i.e. move the axle the
# wrong way. KP and KI are therefore NEGATIVE.
SMC_KP = -0.30               # rad per m/s; bandwidth 1.8 rad/s with the static gain a1 - b1 a2 / b2
SMC_KI = -0.10               # rad per m
TH_MAX = 0.21                # rad (12 deg): past ~15 deg the linearised VL-WIP model loses validity
K_POS = 1.2                  # 1/s: position error folded into the velocity reference (drive/planar runs)
V_REF_MAX = 2.0              # m/s: ceiling of the whole outer loop (velocity + position correction)

# ---------------- pitch: super-twisting (second-order sliding mode) ----------------
SMC_C1 = 10.0                # s1 = theta_dot + c1 (theta - theta*)
SMC_KA = 25.0                # sqrt term: reaching time ~0.08 s from s1 = 1
SMC_KB = 250.0               # integral term
SMC_EPS1 = 0.02              # s/(|s|+eps) instead of sign(s): continuous, so no chattering
SMC_W_MAX = 60.0             # rad/s^2, safety clamp on the twisting integrator
SMC_DT_MAX = 0.004           # s: a late tick must not integrate kb over the whole gap
# c1 multiplies theta_dot, a filtered numerical derivative (tau = 9 ms). On the linearised plant the loop
# survives c1 = 40 even with sensor noise and actuator lag; what grows with c1 is torque activity, not
# instability. The real limit is the 12.5 Hz torsional resonance of the stance legs, which this model does
# not contain, so raise c1 only while watching the torque spectrum.
WHEEL_TORQUE_MAX = 10.0
MU_SAFE = 0.8                # fraction of the URDF friction coefficient used for the slip limit
N_WHEEL_MIN_RATIO = 0.35     # floor on the per-wheel normal force when the contact estimate dips

# ---------------- yaw: differential wheel torque ----------------
# PD reproducing the closed-loop poles of the previous TV-LQR (-7.45 +- 2.62j) with b3 = 34.9 rad/s^2/Nm.
# Deliberately NOT sliding mode: a discontinuous term here excites the 12.5 Hz torsional resonance of the
# stance legs (torso yaw inertia on the leg springs) into a permanent limit cycle.
YAW_KP = 0.89                # Nm per wheel per rad
YAW_KD = 0.21                # Nm per wheel per rad/s
DIFF_TORQUE_MAX = 2.5        # Nm per wheel for yaw: balance (common mode) keeps priority
# wheel scrub while turning: Coulomb + viscous feedforward from the commanded yaw rate (identified in Gazebo)
# and a slow integral on the heading error; both act far below the leg torsional resonance
YAW_FRICTION = 0.25          # Nm per wheel
YAW_VISCOUS = 0.10           # Nm per wheel per rad/s
YAW_FRICTION_SMOOTH = 0.01   # rad/s (acts on the reference only: no chattering)
YAW_KI = 0.6                 # Nm per wheel per rad s
YAW_I_MAX = 0.6              # Nm per wheel
YAW_I_DEADBAND = 0.035       # rad: no integration on small static errors (avoids stick-slip hunting)
YAW_ERROR_MAX = 0.35         # rad; larger yaw errors are clamped (turn-rate commands the wheels cannot follow)
THETA_DOT_FILTER = 0.8        # first-order filters, weight of the previous value at 500 Hz
S_DOT_FILTER = 0.8

# ---------------- support force ----------------
F_MIN_RATIO = 0.3
F_MAX_RATIO = 3.0
F_MAX_THRUST_RATIO = 6.0

# ---------------- legs: per-leg task-space SMC, base frame (x, z) ----------------
# The old VMC was already a sliding-mode law without the switching term:
#   Kp e + Kd e_dot  ==  -Kd [e_dot + (Kp/Kd) e],  a surface with c = Kp/Kd.
# c and K_lin below reproduce the validated VMC gains exactly, so only the robust term is new.
# Stance keeps a vertical stiffness (c_z * K_lin_z = 1200 N/m): unlike the MPC era, F_z is now a plain
# feedforward, so without it the CoM height would have no closed loop at all.
LEG_C_STANCE = np.array([37.5, 48.0])        # 1/s
LEG_KLIN_STANCE = np.diag([40.0, 25.0])
LEG_C_HOLD = np.array([37.5, 40.0])
LEG_KLIN_HOLD = np.diag([40.0, 50.0])
LEG_C_FLIGHT = np.array([40.0, 40.0])
LEG_KLIN_FLIGHT = np.diag([20.0, 20.0])
LEG_KSW = 8.0                # N, robust term (det J = 0.0169 m^2 -> ~11 N per Nm)
LEG_PHI = 0.03               # m/s, boundary layer
LEG_TORQUE_MAX = 50.0
FLIGHT_WHEEL_DAMPING = 0.05

# ---------------- Kalman filter (eqs. 20-21) ----------------
KF_Q_ACC = 0.5
KF_R_POS = 1e-4
KF_R_VEL = 1e-3

# ---------------- contact / jump ----------------
F_CONTACT_OFF = 1.5
F_CONTACT_ON = 6.0
CONTACT_LOSS_DEBOUNCE = 0.004
CONTACT_GAIN_DEBOUNCE = 0.010
CONTACT_STALE_TIME = 0.006

HIP_AXLE_LOW = 0.12          # squat hip-axle distance [m]
HIP_AXLE_HIGH = 0.24         # thrust target [m]
HIP_AXLE_FLIGHT = 0.17
T_SQUAT = 0.8
T_THRUST_MAX = 0.30
T_LANDING = 1.0
T_SETTLE = 1.5

PHASE_HOLD = 'HOLD'
PHASE_BALANCE = 'BALANCE'
PHASE_SQUAT = 'PRELOAD'
PHASE_THRUST = 'THRUST'
PHASE_FLIGHT = 'FLIGHT'
PHASE_LANDING = 'LANDING'
STANCE_PHASES = (PHASE_BALANCE, PHASE_SQUAT, PHASE_THRUST, PHASE_LANDING)

PARAMS = {
    'jump_enable': False, 'jump_start_time': 4.0, 'jump_velocity': 1.2,
    'height_enable': False, 'height_amplitude': 0.03, 'height_period': 2.2,
    'velocity_enable': False, 'velocity_max': 0.44, 'accel_max': 0.6, 'velocity_distance': 2.0,
    'drive_enable': False, 'drive_distance': 1.0, 'drive_period': 8.0,
    'planar_enable': False, 'traj_length': 3.0, 'traj_lateral': 0.6, 'traj_duration': 12.0,
    'push_enable': False, 'push_time': 4.0, 'push_impulse': 2.7,
}

K_LAT = 2.0
LAT_FULL_SPEED = 0.10

# ---------------- live commands (/rotino/cmd_*) ----------------
CMD_TIMEOUT = 0.5            # s without /rotino/cmd_vel -> velocity and turn rate go back to zero
CMD_V_MAX = 2.0              # m/s (mirrored in rotino_dashboard/dashboard.py for the slider range)
CMD_W_MAX = 1.5              # rad/s
CMD_YAW_ACC = 3.0            # rad/s^2 (forward acceleration uses accel_max)
CMD_HEIGHT_MIN, CMD_HEIGHT_MAX = -0.05, 0.04   # m, offset of the CoM height from the nominal pose
CMD_HEIGHT_RATE = 0.08       # m/s
CMD_PUSH_MAX = 6.0           # N s


def smoothstep(t, T):
    tau = min(max(t / T, 0.0), 1.0) if T > 0.0 else 1.0
    return 3.0 * tau ** 2 - 2.0 * tau ** 3, (6.0 * tau - 6.0 * tau ** 2) / T if T > 0.0 else 0.0


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class WBRController(Node):

    def __init__(self):
        super().__init__('rotino_smc_controller')
        self.declare_parameter('robot_description', '')
        for name, default in PARAMS.items():
            self.declare_parameter(name, default)
        self.cfg = {name: self.get_parameter(name).value for name in PARAMS}
        urdf_xml = self.get_parameter('robot_description').value
        if not urdf_xml:
            raise RuntimeError('Parameter robot_description is empty.')

        self.model = WBRModel(urdf_xml)
        p = self.model.p
        self.weight = p.m_b * G

        # height <-> hip angle (hip = -knee/2), used for the pendulum-length grid and the jump heights
        samples = []
        self.height_table = []
        for hip in np.linspace(-0.45, 0.45, 25):
            e = self.model.equivalent_centroid(hip, -2.0 * hip)
            samples.append((e['l'], e['I_y']))
            self.height_table.append((e['z_b'], e['Z_C']))
        self.height_table.sort()
        # I_y(l) interpolated rather than recomputed: equivalent_centroid walks every link and builds the
        # inertia tensor, far too slow for 500 Hz. I_z barely moves with the pose, so the nominal one is kept.
        samples.sort()
        self.l_grid = np.array([s[0] for s in samples])
        self.iy_grid = np.array([s[1] for s in samples])
        nominal = self.model.equivalent_centroid(0.0, 0.0)
        self.I_z_nom = nominal['I_z']
        self.z_nom = nominal['Z_C']
        self.z_low = self._z_for_hip_axle(HIP_AXLE_LOW)
        self.z_high = self._z_for_hip_axle(HIP_AXLE_HIGH)
        self.z_flight = self._z_for_hip_axle(HIP_AXLE_FLIGHT)
        self.trajectory = (CubicSTrajectory(self.cfg['traj_length'], self.cfg['traj_lateral'],
                                            self.cfg['traj_duration']) if self.cfg['planar_enable'] else None)

        c_nom = self._vlwip(self.z_nom)
        self.get_logger().info(
            f"m_b={p.m_b:.3f} kg, z_nom={self.z_nom:.4f} m, VL-WIP at l=z_nom: a2={c_nom['a2']:+.1f} 1/s^2, "
            f"b2={c_nom['b2']:+.1f} rad/s^2/Nm, slip limit {MU_SAFE * p.mu * 0.5 * self.weight * p.r:.2f} Nm/wheel, "
            f"squat/thrust CoM heights {self.z_low:.3f}/{self.z_high:.3f} m, modes: "
            + ', '.join(k for k, v in self.cfg.items() if k.endswith('_enable') and v))

        self.wheel_pub = self.create_publisher(Float64MultiArray, '/wheel_effort_controller/commands', 10)
        self.leg_pub = self.create_publisher(Float64MultiArray, '/leg_effort_controller/commands', 10)
        self.release_pub = self.create_publisher(Empty, '/rotino/release', 10)
        self.state_pub = self.create_publisher(String, '/rotino/jump_state', 10)
        self.wrench_pub = self.create_publisher(EntityWrench, '/world/rotino_world/wrench', 10)
        # [t, theta, theta_dot, s, s_dot, wheel_u, com_z, com_z_vel, Fn_total, min_wheel_gap, loaded]
        self.debug_pub = self.create_publisher(Float64MultiArray, '/rotino/debug', 10)
        # [t, s, s_ref, theta, theta_ref, phi, phi_ref, s_dot, s_dot_ref, z, z_ref, s1, F_z, tau_l, tau_r, l,
        #  tau_hip_l, tau_knee_l]   -- s1 is the pitch sliding variable (was delta_s in the MPC version)
        self.wbr_pub = self.create_publisher(Float64MultiArray, '/rotino/wbr_state', 10)
        # [ex, ey, ez, evx, evy, evz]: estimated - ground-truth torso position/velocity (world)
        self.est_err_pub = self.create_publisher(Float64MultiArray, '/rotino/estimation_error', 10)
        self.planar_pub = self.create_publisher(Float64MultiArray, '/rotino/planar', 10)
        self.tracking_error_pub = self.create_publisher(Float64MultiArray, '/rotino/tracking_error', 10)

        self.imu = None
        self.odom = None
        self.contact = {'left': (-math.inf, 0.0), 'right': (-math.inf, 0.0)}
        # depth 1: only the newest sample matters; late control ticks use the real dt
        self.create_subscription(Imu, '/rotino/imu', lambda m: setattr(self, 'imu', m), 1)
        self.create_subscription(Odometry, '/rotino/odom', lambda m: setattr(self, 'odom', m), 1)
        self.create_subscription(Contacts, '/rotino/left_wheel_contact', lambda m: self._contact_cb(m, 'left'), 1)
        self.create_subscription(Contacts, '/rotino/right_wheel_contact', lambda m: self._contact_cb(m, 'right'), 1)
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 1)
        self.create_subscription(Twist, '/rotino/cmd_vel', self._cmd_vel_cb, 10)
        self.create_subscription(Float64, '/rotino/cmd_height', self._cmd_height_cb, 10)
        self.create_subscription(Empty, '/rotino/cmd_jump', self._cmd_jump_cb, 10)
        self.create_subscription(Float64, '/rotino/cmd_push', self._cmd_push_cb, 10)

        self.kf = [AxisKalman(KF_Q_ACC, KF_R_POS, KF_R_VEL) for _ in range(3)]
        self.phase = PHASE_HOLD
        self.phase_t0 = 0.0
        self.hold_start = None
        self.t_release = None
        self.last_stamp = None
        self.tick = 0
        self.cpu_ms = 0.0
        self.max_dt = 0.0
        self.last_print = -math.inf
        self.jump_done = False
        self.jump_request = False
        self.jump_wait_logged = False
        self.push_request = None
        self.push_ticks_left = 0
        self.push_force = 0.0
        self.scripted_push_done = False
        self.heading_hold = 0.0

        # live commands: once one arrives the scripted motion profiles are replaced by the teleop reference
        self.teleop = False
        self.teleop_ready = False
        self.cmd_v = self.cmd_w = self.cmd_height = 0.0
        self.cmd_wall = -math.inf
        self.tele_s = self.tele_v = self.tele_yaw = self.tele_w = 0.0
        self.tele_z = None
        self.tele_zd = 0.0
        self.yaw_i = 0.0

        self.loaded = True
        self.unloaded_timer = 0.0
        self.loaded_timer = 0.0
        self.P_w = np.zeros(3)
        self.s = 0.0
        self.s_offset = 0.0
        self.yaw0 = 0.0
        self.prev_theta = None
        self.theta_dot = 0.0
        self.Iv = 0.0            # outer-loop velocity integral
        self.W = 0.0             # super-twisting integrator
        self.s1 = 0.0            # pitch sliding variable (telemetry only)
        self.leg_s = 0.0         # left-leg sliding variable, norm (telemetry only)
        self.s_dot_f = 0.0
        self.F_z = self.weight
        self.z_land = self.z_nom
        self.z_ref_last = self.z_nom
        self.odom0 = None
        self.get_logger().info('Waiting for /joint_states and /rotino/imu ...')

    # -----------------------------------------------------------------
    def _z_for_hip_axle(self, zb):
        zbs, zcs = zip(*self.height_table)
        return float(np.interp(zb, zbs, zcs))

    def _vlwip(self, l):
        """Eq. (14) coefficients at the current pendulum length, I_y read off the pose grid."""
        return self.model.vlwip_coefficients(l, I_y=float(np.interp(l, self.l_grid, self.iy_grid)),
                                             I_z=self.I_z_nom)

    def _contact_cb(self, msg, side):
        force, touching = 0.0, False
        for c in msg.contacts:
            if 'ground' not in c.collision1.name and 'ground' not in c.collision2.name:
                continue
            touching = True
            for w in c.wrenches:
                f = w.body_1_wrench.force
                force += math.sqrt(f.x * f.x + f.y * f.y + f.z * f.z)
            if not c.wrenches:
                force = 0.5 * self.weight
        if touching:
            self.contact[side] = (stamp_to_sec(msg.header.stamp), force)

    # -----------------------------------------------------------------
    # Live commands
    # -----------------------------------------------------------------
    def _start_teleop(self):
        if not self.teleop:
            self.teleop = True
            self.get_logger().info('Live commands received: scripted motion profiles disabled')

    def _cmd_vel_cb(self, msg):
        self.cmd_v = float(np.clip(msg.linear.x, -CMD_V_MAX, CMD_V_MAX))
        self.cmd_w = float(np.clip(msg.angular.z, -CMD_W_MAX, CMD_W_MAX))
        self.cmd_wall = time.monotonic()
        self._start_teleop()

    def _cmd_height_cb(self, msg):
        self.cmd_height = float(np.clip(msg.data, CMD_HEIGHT_MIN, CMD_HEIGHT_MAX))
        self._start_teleop()

    def _cmd_jump_cb(self, _msg):
        if self.phase == PHASE_BALANCE and not self.jump_request:
            self.jump_request = True
            self.jump_wait_logged = False
            self.get_logger().info('Jump requested')

    def _cmd_push_cb(self, msg):
        self.push_request = float(np.clip(msg.data, -CMD_PUSH_MAX, CMD_PUSH_MAX))

    def _update_teleop(self, t, dt, yaw):
        if not self.teleop or self.phase == PHASE_HOLD:
            return
        if not self.teleop_ready:
            # start from the current state so that taking over is bumpless
            self.tele_s = self.s
            self.tele_yaw = wrap(yaw - self.yaw0)
            self.tele_z = self.z_ref_last
            self.teleop_ready = True
        balance = self.phase == PHASE_BALANCE
        fresh = time.monotonic() - self.cmd_wall < CMD_TIMEOUT
        v_target = self.cmd_v if fresh and balance and not self.jump_request else 0.0
        w_target = self.cmd_w if fresh and balance and not self.jump_request else 0.0
        a = self.cfg['accel_max'] * dt
        self.tele_v += float(np.clip(v_target - self.tele_v, -a, a))
        self.tele_w += float(np.clip(w_target - self.tele_w, -CMD_YAW_ACC * dt, CMD_YAW_ACC * dt))
        if balance:
            self.tele_s += self.tele_v * dt
            self.tele_yaw += self.tele_w * dt
            # anti-windup: the heading reference never runs further than YAW_ERROR_MAX ahead of the robot
            lag = wrap(self.tele_yaw - (yaw - self.yaw0))
            if abs(lag) > YAW_ERROR_MAX:
                self.tele_yaw -= lag - math.copysign(YAW_ERROR_MAX, lag)
            step = float(np.clip(self.z_nom + self.cmd_height - self.tele_z, -CMD_HEIGHT_RATE * dt, CMD_HEIGHT_RATE * dt))
            self.tele_z += step
            self.tele_zd = step / dt

    @property
    def z_stand(self):
        return self.tele_z if self.teleop and self.tele_z is not None else self.z_nom

    def _set_phase(self, phase, t):
        self.phase, self.phase_t0 = phase, t
        # both integrators hold the history of a different contact condition; dropping them at the
        # transition avoids a torque step
        self.Iv = 0.0
        self.W = 0.0
        self.state_pub.publish(String(data=phase))
        self.get_logger().info(f'>>> PHASE {phase} t={t:.3f}s')

    # -----------------------------------------------------------------
    # Operator / trajectory references (Fig. 4, blue blocks)
    # -----------------------------------------------------------------
    def _motion_reference(self, t):
        """Returns (s_ref, s_dot_ref, yaw_offset, yaw_rate_ref) of the stance trajectory at time t after release."""
        c = self.cfg
        tm = t - MOTION_START_TIME
        if self.phase != PHASE_BALANCE:
            return 0.0, 0.0, self.heading_hold, 0.0
        if self.teleop and self.teleop_ready:
            return self.tele_s - self.s_offset, self.tele_v, self.tele_yaw, self.tele_w
        if self.jump_done or tm <= 0.0:
            return 0.0, 0.0, self.heading_hold, 0.0
        if c['velocity_enable']:
            v, a, D = c['velocity_max'], c['accel_max'], c['velocity_distance']
            v = min(v, math.sqrt(a * D))
            ta = v / a
            tc = max(0.0, (D - v * ta) / v)
            if tm < ta:
                return 0.5 * a * tm * tm, a * tm, 0.0, 0.0
            if tm < ta + tc:
                return 0.5 * v * ta + v * (tm - ta), v, 0.0, 0.0
            td = min(tm - ta - tc, ta)
            return 0.5 * v * ta + v * tc + v * td - 0.5 * a * td * td, v - a * td, 0.0, 0.0
        if c['drive_enable']:
            w = 2.0 * math.pi / c['drive_period']
            return (0.5 * c['drive_distance'] * (1.0 - math.cos(w * tm)),
                    0.5 * c['drive_distance'] * w * math.sin(w * tm), 0.0, 0.0)
        if self.trajectory is not None:
            s, v = self.trajectory._time_law(tm)
            _, _, heading, _, yaw_rate = self.trajectory.sample(tm)
            return s, v, heading, yaw_rate
        return 0.0, 0.0, 0.0, 0.0

    def _height_reference(self, t):
        """(z_ref, z_dot_ref, z_ddot_ref) of the CoM above the axle at time t after release."""
        tp = t - self.phase_t0
        if self.phase == PHASE_SQUAT:
            a, da = smoothstep(tp, T_SQUAT)
            z0 = self.z_stand
            return z0 + (self.z_low - z0) * a, (self.z_low - z0) * da, 0.0
        if self.phase == PHASE_THRUST:
            # constant acceleration so that the CoM leaves full extension at jump_velocity
            v = self.cfg['jump_velocity']
            acc = v * v / (2.0 * (self.z_high - self.z_low))
            T = v / acc
            if tp < T:
                return self.z_low + 0.5 * acc * tp * tp, acc * tp, acc
            return self.z_high + v * (tp - T), v, 0.0
        if self.phase == PHASE_LANDING:
            a, da = smoothstep(tp, T_LANDING)
            z1 = self.z_stand
            return self.z_land + (z1 - self.z_land) * a, (z1 - self.z_land) * da, 0.0
        if self.teleop and self.tele_z is not None and self.phase == PHASE_BALANCE:
            return self.tele_z, self.tele_zd, 0.0
        th = t - MOTION_START_TIME
        if self.cfg['height_enable'] and th > 0.0 and self.phase == PHASE_BALANCE:
            w = 2.0 * math.pi / self.cfg['height_period']
            A = self.cfg['height_amplitude']
            return self.z_nom + A * math.sin(w * th), A * w * math.cos(w * th), -A * w * w * math.sin(w * th)
        return self.z_nom, 0.0, 0.0

    # -----------------------------------------------------------------
    def _joint_state_cb(self, msg):
        if self.imu is None:
            return
        t_abs = stamp_to_sec(msg.header.stamp)
        if self.last_stamp is None:
            self.last_stamp = t_abs
            return
        dt = t_abs - self.last_stamp
        if dt <= 0.0:
            return
        self.last_stamp = t_abs
        q = dict(zip(msg.name, msg.position))
        qd = dict(zip(msg.name, msg.velocity))
        if any(n not in q for n in LEG_JOINTS + WHEEL_JOINTS):
            return
        self.tick += 1
        t_cpu = time.perf_counter()
        if self.phase == PHASE_HOLD:
            self._hold_step(t_abs, q, qd)
        else:
            self._control_step(t_abs - self.t_release, dt, q, qd)
        self.cpu_ms = 0.99 * self.cpu_ms + 0.01 * (time.perf_counter() - t_cpu) * 1e3
        self.max_dt = max(self.max_dt, dt)

    # -----------------------------------------------------------------
    def _legs(self, q, qd):
        pl, Jl = self.model.leg_fk(q['left_hip'], q['left_knee'])
        pr, Jr = self.model.leg_fk(q['right_hip'], q['right_knee'])
        vl = Jl @ np.array([qd['left_hip'], qd['left_knee']])
        vr = Jr @ np.array([qd['right_hip'], qd['right_knee']])
        self.leg_qd = (np.array([qd['left_hip'], qd['left_knee']]), np.array([qd['right_hip'], qd['right_knee']]))
        return (pl, Jl, vl), (pr, Jr, vr)

    def _leg_smc(self, legs, p_d, c, K_lin, F_ff=np.zeros(2), v_d=np.zeros(2)):
        """Task-space sliding mode, one independent controller per leg.

        Symmetry hypothesis: both legs share the reference and the gains, each closes the loop on its own
        forward kinematics and Jacobian. s = (v_f - v_d) + c (p_f - p_d); the switching term is what the
        old VMC lacked. -> [hip_L, hip_R, knee_L, knee_R]
        """
        taus = []
        for i, ((p_f, J, v_f), qd) in enumerate(zip(legs, self.leg_qd)):
            s = (v_f - v_d) + c * (p_f - p_d)
            F = -K_lin @ s - LEG_KSW * np.clip(s / LEG_PHI, -1.0, 1.0) + F_ff
            tau_ff = self.model.p.leg_damping * qd   # feedforward: cancels the URDF joint damping
            taus.append(np.clip(J.T @ F + tau_ff, -LEG_TORQUE_MAX, LEG_TORQUE_MAX))
            if i == 0:
                self.leg_s = float(np.linalg.norm(s))
        tau = [taus[0][0], taus[1][0], taus[0][1], taus[1][1]]
        self.leg_pub.publish(Float64MultiArray(data=[float(x) for x in tau]))
        return tau

    def _outer_velocity_loop(self, v, v_ref, dt, active):
        """PI on the axle velocity error -> pitch reference. Non-minimum phase, hence KP, KI < 0.

        Deliberately the AXLE velocity, not the CoM one: V_com carries an omega x c_b term, so a pitch
        oscillation shows up in it as a spurious +-0.4 m/s of travel. Above ~0.7 m/s the outer loop locks
        onto that and drives a permanent limit cycle.
        """
        if not active:
            self.Iv = 0.0
            return 0.0
        ev = v - v_ref
        th_raw = SMC_KP * ev + SMC_KI * self.Iv
        th_ref = float(np.clip(th_raw, -TH_MAX, TH_MAX))
        if abs(th_raw - th_ref) < 1e-9:      # anti-windup: stop integrating once theta* saturates
            self.Iv += ev * dt
        return th_ref

    def _pitch_super_twisting(self, theta, theta_ref, l):
        """Second-order sliding mode on the pitch -> common wheel torque.

        theta_ddot = a2 theta + b2 (tau_l + tau_r) (eq. 13) inverts exactly, so nothing is approximated
        between the sliding law and the torque. b2 < 0, so the sign flips on its own.
        """
        c = self._vlwip(l)
        s1 = self.theta_dot + SMC_C1 * (theta - theta_ref)
        sg1 = s1 / (abs(s1) + SMC_EPS1)
        nu = -SMC_C1 * self.theta_dot - SMC_KA * math.sqrt(abs(s1)) * sg1 + self.W
        return 0.5 * (nu - c['a2'] * theta) / c['b2'], s1, sg1

    def _yaw_torque(self, yaw, yaw_ref, yaw_rate, yaw_rate_ref, dt):
        """Differential wheel torque: PD + scrub feedforward + slow integral on the heading error."""
        err = float(np.clip(wrap(yaw - yaw_ref), -YAW_ERROR_MAX, YAW_ERROR_MAX))
        if self.phase != PHASE_BALANCE:
            self.yaw_i = 0.0
        elif abs(err) > YAW_I_DEADBAND or yaw_rate_ref != 0.0:
            self.yaw_i = float(np.clip(self.yaw_i - YAW_KI * err * dt, -YAW_I_MAX, YAW_I_MAX))
        ff = YAW_FRICTION * math.tanh(yaw_rate_ref / YAW_FRICTION_SMOOTH) + YAW_VISCOUS * yaw_rate_ref
        diff = -YAW_KP * err - YAW_KD * (yaw_rate - yaw_rate_ref) + ff + self.yaw_i
        return float(np.clip(diff, -DIFF_TORQUE_MAX, DIFF_TORQUE_MAX))

    def _publish_wheels(self, tau_l, tau_r):
        self.wheel_pub.publish(Float64MultiArray(data=[float(tau_l), float(tau_r)]))

    def _hold_step(self, t_abs, q, qd):
        if self.hold_start is None:
            self.hold_start = t_abs
            self.get_logger().info('Holding on the anchor, legs to the nominal pose ...')
        legs = self._legs(q, qd)
        c_b = self.model.upper_body_com_base(q)
        p_d = np.array([c_b[0], c_b[2] - self.z_nom])
        self._leg_smc(legs, p_d, LEG_C_HOLD, LEG_KLIN_HOLD)
        self._publish_wheels(0.0, 0.0)
        if t_abs - self.hold_start < HOLD_TIME:
            return

        R = quat_to_matrix(*self._quat())
        axle_b = np.array([0.5 * (legs[0][0][0] + legs[1][0][0]), 0.0, 0.5 * (legs[0][0][1] + legs[1][0][1])])
        self.P_w = np.array([0.0, 0.0, self.model.p.r])
        P_b = self.P_w - R @ axle_b
        for i in range(3):
            self.kf[i].reset(P_b[i], 0.0)
        heading = R[:, 0]
        self.yaw0 = math.atan2(heading[1], heading[0])
        if self.odom is not None:
            o = self.odom.pose.pose.position
            self.odom0 = np.array([o.x, o.y, o.z]) - P_b
        self.release_pub.publish(Empty())
        self.t_release = t_abs
        self._set_phase(PHASE_BALANCE, 0.0)

    def _quat(self):
        o = self.imu.orientation
        return o.x, o.y, o.z, o.w

    # -----------------------------------------------------------------
    def _control_step(self, t, dt, q, qd):
        p = self.model.p
        if t < RELEASE_REPEAT_TIME:
            self.release_pub.publish(Empty())

        # ---------------- IMU ----------------
        R = quat_to_matrix(*self._quat())
        gyro_b = np.array([self.imu.angular_velocity.x, self.imu.angular_velocity.y, self.imu.angular_velocity.z])
        omega_w = R @ gyro_b
        f_b = np.array([self.imu.linear_acceleration.x, self.imu.linear_acceleration.y,
                        self.imu.linear_acceleration.z])
        acc_w = R @ f_b - np.array([0.0, 0.0, G])
        heading = R[:, 0].copy()
        heading[2] = 0.0
        heading /= max(np.linalg.norm(heading), 1e-9)
        yaw = math.atan2(heading[1], heading[0])

        # ---------------- leg kinematics, equivalent centroid (eqs. 2-3) ----------------
        legs = self._legs(q, qd)
        axle_b = np.array([0.5 * (legs[0][0][0] + legs[1][0][0]), 0.0, 0.5 * (legs[0][0][1] + legs[1][0][1])])
        v_rel_b = np.array([0.5 * (legs[0][2][0] + legs[1][2][0]), 0.0, 0.5 * (legs[0][2][1] + legs[1][2][1])])
        c_b = self.model.upper_body_com_base(q)
        com_rel_w = R @ (c_b - axle_b)
        S_C = float(np.dot(com_rel_w[:2], heading[:2]))
        Z_C = float(com_rel_w[2])
        l = math.hypot(S_C, Z_C)
        theta = math.atan2(S_C, Z_C)
        if self.prev_theta is None:
            self.prev_theta = theta
        self.theta_dot = THETA_DOT_FILTER * self.theta_dot + (1 - THETA_DOT_FILTER) * (theta - self.prev_theta) / dt
        self.prev_theta = theta

        # ---------------- contact state ----------------
        fn = sum(f if t_now_ok else 0.0 for f, t_now_ok in
                 ((self.contact[s][1], self.last_stamp - self.contact[s][0] <= CONTACT_STALE_TIME)
                  for s in ('left', 'right')))
        if self.loaded:
            self.unloaded_timer = self.unloaded_timer + dt if fn < F_CONTACT_OFF else 0.0
            if self.unloaded_timer >= CONTACT_LOSS_DEBOUNCE:
                self.loaded, self.loaded_timer = False, 0.0
        else:
            self.loaded_timer = self.loaded_timer + dt if fn > F_CONTACT_ON else 0.0
            if self.loaded_timer >= CONTACT_GAIN_DEBOUNCE:
                self.loaded, self.unloaded_timer = True, 0.0
        stance = self.phase in STANCE_PHASES and self.loaded

        # ---------------- Kalman filter (eqs. 20-21) ----------------
        axle_w_rel = R @ axle_b
        wP_b = -axle_w_rel
        wV_b = -np.cross(omega_w, axle_w_rel) - R @ v_rel_b
        for i in range(3):
            self.kf[i].predict(acc_w[i], dt)
        P_b = np.array([k.x[0] for k in self.kf])
        V_b = np.array([k.x[1] for k in self.kf])
        if stance:
            pitch_rate = gyro_b[1]
            w_l = qd['left_wheel_joint'] + pitch_rate + qd['left_hip'] + qd['left_knee']
            w_r = qd['right_wheel_joint'] + pitch_rate + qd['right_hip'] + qd['right_knee']
            V_w = 0.5 * p.r * (w_l + w_r) * heading
            self.P_w = self.P_w + V_w * dt
            self.P_w[2] = p.r
            for i in range(3):
                self.kf[i].correct(self.P_w[i] + wP_b[i], V_w[i] + wV_b[i])
            P_b = np.array([k.x[0] for k in self.kf])
            V_b = np.array([k.x[1] for k in self.kf])
        else:
            self.P_w = P_b - wP_b
        V_axle = V_b - wV_b
        self.s_dot_f = S_DOT_FILTER * self.s_dot_f + (1 - S_DOT_FILTER) * float(np.dot(V_axle[:2], heading[:2]))
        s_dot = self.s_dot_f
        self.s += float(np.dot(V_axle[:2], heading[:2])) * dt
        com_height = float(P_b[2] + (R @ c_b)[2])
        wheel_gap = float(P_b[2] + axle_w_rel[2]) - p.r

        # ---------------- phase machine (jump: Fig. 11 stages) ----------------
        tp = t - self.phase_t0
        if self.phase == PHASE_BALANCE:
            scripted = self.cfg['jump_enable'] and not self.jump_done and t > self.cfg['jump_start_time']
            if scripted or self.jump_request:
                if abs(self.theta_dot) < 0.3 and abs(s_dot) < 0.08:
                    self.jump_request = False
                    self.s_offset = self.s
                    self.heading_hold = wrap(yaw - self.yaw0)
                    self.tele_v = self.tele_w = 0.0
                    self._set_phase(PHASE_SQUAT, t)
                elif self.jump_request and not self.jump_wait_logged:
                    self.jump_wait_logged = True
                    self.get_logger().info('Jump: stopping the robot first')
        elif self.phase == PHASE_SQUAT and tp >= T_SQUAT:
            self._set_phase(PHASE_THRUST, t)
        elif self.phase == PHASE_THRUST:
            if not self.loaded:
                self._set_phase(PHASE_FLIGHT, t)
            elif tp >= T_THRUST_MAX:
                self.get_logger().warn('No take-off detected')
                self.z_land = Z_C
                self.s_offset = self.tele_s = self.s
                self._set_phase(PHASE_LANDING, t)
        elif self.phase == PHASE_FLIGHT:
            if self.loaded:
                self.z_land = Z_C
                self.s_offset = self.tele_s = self.s
                self.get_logger().info(f'Landed after {tp:.3f}s of flight')
                self._set_phase(PHASE_LANDING, t)
        elif self.phase == PHASE_LANDING and tp >= T_LANDING + T_SETTLE:
            self.jump_done = True
            self._set_phase(PHASE_BALANCE, t)

        # ---------------- references ----------------
        self._update_teleop(t, dt, yaw)
        z_ref, zd_ref, zdd_ref = self._height_reference(t)
        self.z_ref_last = z_ref
        s_ref, sd_ref, heading_ref, yaw_rate_ref = self._motion_reference(t)
        s_ref += self.s_offset
        yaw_ref = self.yaw0 + heading_ref
        if self.trajectory is not None and self.phase == PHASE_BALANCE and not self.jump_done and not self.teleop:
            yaw_ref = self._planar_correction(t, P_b, wP_b, yaw, yaw_ref, sd_ref)

        tau_l = tau_r = 0.0
        tau_legs = [0.0] * 4
        theta_ref = 0.0
        if self.phase == PHASE_FLIGHT:
            # wheels held still, legs retracted under the CoM (flight task-space controller)
            p_d = np.array([c_b[0], c_b[2] - self.z_flight])
            tau_legs = self._leg_smc(legs, p_d, LEG_C_FLIGHT, LEG_KLIN_FLIGHT)
            tau_l = -FLIGHT_WHEEL_DAMPING * qd['left_wheel_joint']
            tau_r = -FLIGHT_WHEEL_DAMPING * qd['right_wheel_joint']
            self.Iv = self.W = 0.0          # nothing to balance against in the air
        else:
            # ---------------- outer loop: position -> velocity -> theta* ----------------
            balancing = self.phase == PHASE_BALANCE and self.loaded
            v_ref = float(np.clip(sd_ref + K_POS * (s_ref - self.s), -V_REF_MAX, V_REF_MAX)) if balancing else 0.0
            theta_ref = self._outer_velocity_loop(s_dot, v_ref, dt, balancing)

            # ---------------- pitch: super-twisting -> common wheel torque ----------------
            common_raw, self.s1, sg1 = self._pitch_super_twisting(theta, theta_ref, l)
            diff = self._yaw_torque(yaw, yaw_ref, omega_w[2], yaw_rate_ref, dt)
            # traction, not the actuator, is what binds here: mu N r is ~2 Nm against WHEEL_TORQUE_MAX = 10
            n_wheel = max(0.5 * fn, N_WHEEL_MIN_RATIO * self.weight)
            lim = max(min(WHEEL_TORQUE_MAX, MU_SAFE * p.mu * n_wheel * p.r) - abs(diff), 0.0)
            common = float(np.clip(common_raw, -lim, lim))
            if abs(common_raw - common) < 1e-9:    # freeze the twisting integrator while saturated
                self.W = float(np.clip(self.W - SMC_KB * sg1 * min(dt, SMC_DT_MAX), -SMC_W_MAX, SMC_W_MAX))
            tau_l, tau_r = common - diff, common + diff

            # ---------------- legs: per-leg task-space SMC ----------------
            f_max = (F_MAX_THRUST_RATIO if self.phase == PHASE_THRUST else F_MAX_RATIO) * self.weight
            self.F_z = float(np.clip(p.m_b * (G + zdd_ref), F_MIN_RATIO * self.weight, f_max))
            up_b = R.T @ np.array([0.0, 0.0, 1.0])
            F_ff = -0.5 * self.F_z * np.array([up_b[0], up_b[2]])
            p_d = np.array([c_b[0], c_b[2] - z_ref])
            v_d = np.array([0.0, -zd_ref])
            tau_legs = self._leg_smc(legs, p_d, LEG_C_STANCE, LEG_KLIN_STANCE, F_ff, v_d)

        self._publish_wheels(tau_l, tau_r)
        self._push(t)

        # ---------------- telemetry ----------------
        if self.tick % 50 == 0:   # 10 Hz: late subscribers (dashboard) also learn the current phase
            self.state_pub.publish(String(data=self.phase))
        self.debug_pub.publish(Float64MultiArray(data=[
            t, theta, self.theta_dot, self.s, s_dot, 0.5 * (tau_l + tau_r) / p.wheel_torque_max,
            com_height, float(V_b[2]), fn, wheel_gap, float(self.loaded)]))
        self.wbr_pub.publish(Float64MultiArray(data=[
            t, self.s, s_ref, theta, theta_ref, yaw, yaw_ref, s_dot, sd_ref,
            Z_C, z_ref, self.s1, self.F_z, float(tau_l), float(tau_r), l, tau_legs[0], tau_legs[2]]))
        if self.odom is not None and self.odom0 is not None:
            o, v = self.odom.pose.pose.position, self.odom.twist.twist.linear
            Ro = quat_to_matrix(self.odom.pose.pose.orientation.x, self.odom.pose.pose.orientation.y,
                                self.odom.pose.pose.orientation.z, self.odom.pose.pose.orientation.w)
            gt_p = np.array([o.x, o.y, o.z]) - self.odom0
            gt_v = Ro @ np.array([v.x, v.y, v.z])
            self.est_err_pub.publish(Float64MultiArray(data=[*(P_b - gt_p), *(V_b - gt_v)]))

        if t - self.last_print >= 0.5:
            self.last_print = t
            self.get_logger().info(
                f't={t:6.2f} {self.phase:>8} th={math.degrees(theta):+6.2f}deg thref={math.degrees(theta_ref):+5.2f} '
                f's={self.s:+6.3f}/{s_ref:+6.3f} v={s_dot:+5.2f}/{sd_ref:+5.2f} z={Z_C:.3f}/{z_ref:.3f} '
                f'yaw={math.degrees(wrap(yaw - self.yaw0)):+6.1f} s1={self.s1:+.3f} W={self.W:+6.1f} Fz={self.F_z:5.1f} '
                f'tau={tau_l:+5.2f}/{tau_r:+5.2f} legs={tau_legs[0]:+5.1f}/{tau_legs[2]:+5.1f} '
                f'gap={wheel_gap:+.3f} loaded={int(self.loaded)} cpu={self.cpu_ms:.2f}ms max_dt={self.max_dt * 1e3:.0f}ms')
            self.max_dt = 0.0

    # -----------------------------------------------------------------
    def _planar_correction(self, t, P_b, wP_b, yaw, yaw_ref, v_ref):
        tm = max(t - MOTION_START_TIME, 0.0)
        px, py, _, _, _ = self.trajectory.sample(tm)
        c0, s0 = math.cos(self.yaw0), math.sin(self.yaw0)
        p_ref = np.array([c0 * px - s0 * py, s0 * px + c0 * py])
        axle_xy = (P_b - wP_b)[:2]
        err = axle_xy - p_ref
        normal = np.array([-math.sin(yaw_ref), math.cos(yaw_ref)])
        lateral = float(np.dot(err, normal))
        yaw_cmd = yaw_ref - math.atan(K_LAT * lateral) * min(1.0, abs(v_ref) / LAT_FULL_SPEED)
        self.planar_pub.publish(Float64MultiArray(data=[
            t, float(p_ref[0]), float(p_ref[1]), yaw_ref, float(axle_xy[0]), float(axle_xy[1]), yaw,
            float(np.dot(err, [math.cos(yaw_ref), math.sin(yaw_ref)])), lateral, 0.0]))
        self.tracking_error_pub.publish(Float64MultiArray(data=[float(err[0]), float(err[1])]))
        return yaw_cmd

    def _push(self, t):
        """Horizontal impulse on the torso: positive = forward along the heading."""
        c = self.cfg
        if c['push_enable'] and not self.scripted_push_done and t >= c['push_time']:
            self.scripted_push_done = True
            self.push_request = -c['push_impulse']
        if self.push_request is not None and self.push_ticks_left == 0:
            self.push_force = self.push_request / (PUSH_TICKS * PHYSICS_DT)
            self.push_ticks_left = PUSH_TICKS
            self.get_logger().info(f'>>> PUSH {self.push_request:+.2f} N s at t={t:.3f}s')
            self.push_request = None
        if self.push_ticks_left == 0:
            return
        self.push_ticks_left -= 1
        msg = EntityWrench()
        msg.entity.name = 'rotino::base_link'
        msg.entity.type = Entity.LINK
        heading = quat_to_matrix(*self._quat())[:, 0]
        msg.wrench.force.x = float(self.push_force * heading[0])
        msg.wrench.force.y = float(self.push_force * heading[1])
        self.wrench_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WBRController()
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
