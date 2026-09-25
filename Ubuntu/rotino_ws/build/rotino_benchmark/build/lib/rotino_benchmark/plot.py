"""Generates publication-quality comparison plots from CSV files produced by rotino_benchmark.

Follows the academic style of FSR Homework 4:
- Tracking & Reference overlay (dashed for reference, solid for actual)
- Dedicated tracking error time-series
- Disturbance analysis with Phase Portraits (theta vs theta_dot)
- Initial/Disturbance impact markers and sliding surface s1 = 0 for SMC

Usage:
    ros2 run rotino_benchmark plot -- <run_dir>
    python3 src/rotino_benchmark/rotino_benchmark/plot.py <run_dir>
"""

import argparse
import csv
import glob
import math
import os
import sys

try:
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
except ImportError:
    print('matplotlib is required for plotting: pip install matplotlib')
    sys.exit(1)

CONTROLLERS = ('mpc', 'smc', 'pid')
COLORS = {'mpc': '#1f77b4', 'smc': '#2ca02c', 'pid': '#d62728'}
LINESTYLES = {'mpc': '-', 'smc': '-', 'pid': '-'}


def load_run_data(run_dir):
    data = {}
    for law in CONTROLLERS:
        files = sorted(glob.glob(os.path.join(run_dir, f'rotino_{law}_*.csv')))
        if not files:
            continue
        with open(files[-1], newline='') as f:
            reader = csv.DictReader(f)
            t, th, th_d, u, x, v, z, fn = [], [], [], [], [], [], [], []
            th_ref, th_err = [], []
            s_ref, s_err = [], []
            v_ref, v_err = [], []
            z_ref, z_err = [], []
            s1, push_f = [], []
            fn_left, fn_right = [], []
            hip_pos, knee_pos = [], []

            for r in reader:
                try:
                    ti = float(r['time_s'])
                    thi = float(r['theta_deg'])
                    thdi = float(r.get('theta_dot_degs', 0.0))
                    ui = 0.5 * (float(r['wheel_L_torque_cmd']) + float(r['wheel_R_torque_cmd']))
                    xi = float(r['x_m'])
                    vi = float(r['xdot_ms'])
                    zi = float(r['com_z_m'])
                    fni = float(r['fn_total_N'])

                    # References and errors (if present in updated CSV)
                    th_r = float(r.get('theta_ref_deg', 0.0))
                    th_e = float(r.get('theta_err_deg', thi - th_r))
                    sr = float(r.get('s_ref_m', 0.0))
                    se = float(r.get('s_err_m', xi - sr))
                    vr = float(r.get('xdot_ref_ms', 0.0))
                    ve = float(r.get('xdot_err_ms', vi - vr))
                    zr = float(r.get('com_z_ref_m', 0.22))
                    ze = float(r.get('com_z_err_mm', (zi - zr) * 1000.0))
                    s1_val = float(r.get('smc_s1', float('nan')))
                    pf = float(r.get('push_force_N', 0.0))

                    t.append(ti)
                    th.append(thi)
                    th_d.append(thdi)
                    u.append(ui)
                    x.append(xi)
                    v.append(vi)
                    z.append(zi)
                    fn.append(fni)
                    th_ref.append(th_r)
                    th_err.append(th_e)
                    s_ref.append(sr)
                    s_err.append(se)
                    v_ref.append(vr)
                    v_err.append(ve)
                    z_ref.append(zr)
                    z_err.append(ze)
                    s1.append(s1_val)
                    push_f.append(pf)
                    fn_left.append(float(r.get('fn_left_N', 0.0)))
                    fn_right.append(float(r.get('fn_right_N', 0.0)))
                    hip_pos.append(float(r.get('hip_L_pos', 0.0)))
                    knee_pos.append(float(r.get('knee_L_pos', 0.0)))
                except (ValueError, TypeError):
                    continue

            if t:
                data[law] = {
                    't': t, 'th': th, 'th_d': th_d, 'u': u,
                    'x': x, 'v': v, 'z': z, 'fn': fn,
                    'th_ref': th_ref, 'th_err': th_err,
                    's_ref': s_ref, 's_err': s_err,
                    'v_ref': v_ref, 'v_err': v_err,
                    'z_ref': z_ref, 'z_err': z_err,
                    's1': s1, 'push_f': push_f, 'fn_left': fn_left, 'fn_right': fn_right, 'hip_pos': hip_pos, 'knee_pos': knee_pos, 'hip_u': hip_u, 'knee_u': knee_u, 'roll': roll, 'yaw': yaw,
                }
    return data


def plot_tracking_and_errors(data, out_dir):
    """Layout A: Tracking overlay (dashed ref vs solid actual) and dedicated error subplots (Homework 4 style)."""
    fig, axes = plt.subplots(4, 2, figsize=(14, 11), sharex='col')

    # Column 0: Actual vs Reference
    # 0,0: Pitch
    for law, d in data.items():
        c = COLORS.get(law, '#333')
        axes[0, 0].plot(d['t'], d['th'], label=f'{law.upper()} actual', color=c, lw=1.2)
    # Reference
    first_law = next(iter(data))
    axes[0, 0].plot(data[first_law]['t'], data[first_law]['th_ref'],
                    label=r'$\theta^*$ desired', color='black', linestyle='--', lw=1.3, alpha=0.8)
    axes[0, 0].set_ylabel('Pitch [deg]', fontsize=10)
    axes[0, 0].set_title('State vs Desired Reference Trajectory', fontsize=12, fontweight='bold')
    axes[0, 0].grid(True, linestyle='--', alpha=0.5)
    axes[0, 0].legend(loc='upper right', fontsize=8)

    # 1,0: Position x
    for law, d in data.items():
        c = COLORS.get(law, '#333')
        axes[1, 0].plot(d['t'], d['x'], label=f'{law.upper()} actual', color=c, lw=1.2)
    axes[1, 0].plot(data[first_law]['t'], data[first_law]['s_ref'],
                    label=r'$x^*$ desired', color='black', linestyle='--', lw=1.3, alpha=0.8)
    axes[1, 0].set_ylabel('Position x [m]', fontsize=10)
    axes[1, 0].grid(True, linestyle='--', alpha=0.5)
    axes[1, 0].legend(loc='upper left', fontsize=8)

    # 2,0: Velocity x_dot
    for law, d in data.items():
        c = COLORS.get(law, '#333')
        axes[2, 0].plot(d['t'], d['v'], label=f'{law.upper()} actual', color=c, lw=1.2)
    axes[2, 0].plot(data[first_law]['t'], data[first_law]['v_ref'],
                    label=r'$v^*$ desired', color='black', linestyle='--', lw=1.3, alpha=0.8)
    axes[2, 0].set_ylabel('Velocity [m/s]', fontsize=10)
    axes[2, 0].grid(True, linestyle='--', alpha=0.5)
    axes[2, 0].legend(loc='upper right', fontsize=8)

    # 3,0: Height CoM z
    for law, d in data.items():
        c = COLORS.get(law, '#333')
        axes[3, 0].plot(d['t'], d['z'], label=f'{law.upper()} actual', color=c, lw=1.2)
    axes[3, 0].plot(data[first_law]['t'], data[first_law]['z_ref'],
                    label=r'$z^*$ desired', color='black', linestyle='--', lw=1.3, alpha=0.8)
    axes[3, 0].set_ylabel('CoM Height [m]', fontsize=10)
    axes[3, 0].set_xlabel('Time [s]', fontsize=10)
    axes[3, 0].grid(True, linestyle='--', alpha=0.5)
    axes[3, 0].legend(loc='upper right', fontsize=8)

    # Column 1: Tracking Errors
    # 0,1: Pitch Error
    for law, d in data.items():
        c = COLORS.get(law, '#333')
        axes[0, 1].plot(d['t'], d['th_err'], label=f'{law.upper()} $e_\\theta$', color=c, lw=1.2)
    axes[0, 1].set_ylabel(r'Pitch Error $e_\theta$ [deg]', fontsize=10)
    axes[0, 1].set_title('Tracking Errors', fontsize=12, fontweight='bold')
    axes[0, 1].axhline(0, color='gray', linestyle=':', lw=0.8)
    axes[0, 1].grid(True, linestyle='--', alpha=0.5)
    axes[0, 1].legend(loc='upper right', fontsize=8)

    # 1,1: Position Error
    for law, d in data.items():
        c = COLORS.get(law, '#333')
        axes[1, 1].plot(d['t'], d['s_err'], label=f'{law.upper()} $e_x$', color=c, lw=1.2)
    axes[1, 1].set_ylabel(r'Position Error $e_x$ [m]', fontsize=10)
    axes[1, 1].axhline(0, color='gray', linestyle=':', lw=0.8)
    axes[1, 1].grid(True, linestyle='--', alpha=0.5)
    axes[1, 1].legend(loc='upper right', fontsize=8)

    # 2,1: Velocity Error
    for law, d in data.items():
        c = COLORS.get(law, '#333')
        axes[2, 1].plot(d['t'], d['v_err'], label=f'{law.upper()} $e_v$', color=c, lw=1.2)
    axes[2, 1].set_ylabel(r'Velocity Error $e_v$ [m/s]', fontsize=10)
    axes[2, 1].axhline(0, color='gray', linestyle=':', lw=0.8)
    axes[2, 1].grid(True, linestyle='--', alpha=0.5)
    axes[2, 1].legend(loc='upper right', fontsize=8)

    # 3,1: Height Error (mm)
    for law, d in data.items():
        c = COLORS.get(law, '#333')
        axes[3, 1].plot(d['t'], d['z_err'], label=f'{law.upper()} $e_z$', color=c, lw=1.2)
    axes[3, 1].set_ylabel(r'Height Error $e_z$ [mm]', fontsize=10)
    axes[3, 1].set_xlabel('Time [s]', fontsize=10)
    axes[3, 1].axhline(0, color='gray', linestyle=':', lw=0.8)
    axes[3, 1].grid(True, linestyle='--', alpha=0.5)
    axes[3, 1].legend(loc='upper right', fontsize=8)

    plt.tight_layout()
    path = os.path.join(out_dir, 'tracking_and_errors.png')
    plt.savefig(path, dpi=300)
    plt.close()
    return path


def plot_disturbance_and_phase_portrait(data, out_dir):
    """Layout B: Disturbance response & Phase Portrait (Homework 4 Exercise 4 style).

    Left: Time histories of theta(t) and wheel torque tau(t) around disturbance.
    Right: Phase portrait (theta vs theta_dot) with disturbance impact point and recovery orbit.
    """
    fig = plt.figure(figsize=(15, 6.5))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.2, 1.0])

    ax_th = fig.add_subplot(gs[0, 0])
    ax_tau = fig.add_subplot(gs[1, 0], sharex=ax_th)
    ax_phase = fig.add_subplot(gs[:, 1])

    # Find disturbance time if any
    t_push = 4.0
    for law, d in data.items():
        push_indices = [i for i, f in enumerate(d['push_f']) if f > 1.0]
        if push_indices:
            t_push = d['t'][push_indices[0]]
            break

    # Time range: around disturbance [t_push - 1.0, t_push + 6.0]
    t_min = max(0.0, t_push - 1.0)
    t_max = t_push + 6.0

    for law in ('mpc', 'smc'):
        if law not in data:
            continue
        d = data[law]
        c = COLORS[law]

        # Time slice around disturbance
        idx = [i for i, ti in enumerate(d['t']) if t_min <= ti <= t_max]
        if not idx:
            idx = range(len(d['t']))

        t_sub = [d['t'][i] for i in idx]
        th_sub = [d['th'][i] for i in idx]
        u_sub = [d['u'][i] for i in idx]
        thd_sub = [d['th_d'][i] for i in idx]

        # Left Top: Pitch angle response
        ax_th.plot(t_sub, th_sub, label=law.upper(), color=c, lw=1.6)

        # Left Bottom: Commanded wheel torque
        ax_tau.plot(t_sub, u_sub, label=law.upper(), color=c, lw=1.4)

        # Right: Phase Portrait
        ax_phase.plot(th_sub, thd_sub, label=f'{law.upper()} orbit', color=c, lw=1.4, alpha=0.85)

        # Highlight disturbance impact point
        # find closest index to t_push
        i_hit = min(idx, key=lambda i: abs(d['t'][i] - t_push))
        ax_phase.plot(d['th'][i_hit], d['th_d'][i_hit], marker='o', markersize=7,
                      color=c, markeredgecolor='black', markeredgewidth=1.2)

    # Disturbance vertical marker on time plots
    ax_th.axvline(t_push, color='#e74c3c', linestyle='--', lw=1.2, label=f'Disturbance impulse (t={t_push:.1f}s)')
    ax_tau.axvline(t_push, color='#e74c3c', linestyle='--', lw=1.2)

    ax_th.set_title('Disturbance Rejection: Time History', fontsize=12, fontweight='bold')
    ax_th.set_ylabel('Pitch Angle $\\theta$ [deg]', fontsize=10)
    ax_th.grid(True, linestyle='--', alpha=0.5)
    ax_th.legend(loc='upper right', fontsize=8)

    ax_tau.set_title('Control Effort During Disturbance', fontsize=11)
    ax_tau.set_ylabel('Wheel Torque $\\tau$ [Nm]', fontsize=10)
    ax_tau.set_xlabel('Time [s]', fontsize=10)
    ax_tau.grid(True, linestyle='--', alpha=0.5)
    ax_tau.legend(loc='upper right', fontsize=8)

    # Right: Phase portrait styling
    ax_phase.set_title(r'Phase Portrait: $\theta$ vs $\dot{\theta}$', fontsize=12, fontweight='bold')
    ax_phase.set_xlabel(r'Pitch $\theta$ [deg]', fontsize=10)
    ax_phase.set_ylabel(r'Pitch Rate $\dot{\theta}$ [deg/s]', fontsize=10)
    ax_phase.grid(True, linestyle='--', alpha=0.5)
    ax_phase.axhline(0, color='gray', linestyle=':', lw=0.8)
    ax_phase.axvline(0, color='gray', linestyle=':', lw=0.8)

    # Plot the Sliding Surface s1 = theta_dot + c1 * theta = 0 for SMC
    # In rotino_smc, c1 = 12.0
    th_range = ax_phase.get_xlim()
    th_vals = [-10.0, 10.0]
    c1 = 10.0  # nominal sliding slope
    thd_vals = [-c1 * val for val in th_vals]
    ax_phase.plot(th_vals, thd_vals, color='#8e44ad', linestyle='-.', lw=1.3,
                  label=r'Sliding surface $s_1 = \dot{\theta} + c_1 \theta = 0$')

    # Custom legend for phase portrait
    handles, labels = ax_phase.get_legend_handles_labels()
    handles.append(Line2D([0], [0], marker='o', color='w', markerfacecolor='black',
                          markeredgecolor='black', markersize=7, label='Disturbance impact'))
    ax_phase.legend(handles=handles, loc='upper right', fontsize=8)

    plt.tight_layout()
    path = os.path.join(out_dir, 'disturbance_and_phase_portrait.png')
    plt.savefig(path, dpi=300)
    plt.close()
    return path




def plot_ground_reaction_forces_and_joints(data, out_dir):
    """Layout D: Ground reaction forces & VMC Joint action."""
    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    
    # 1. Ground Reaction Forces
    for law in ('mpc', 'smc'):
        if law not in data:
            continue
        d = data[law]
        c = COLORS.get(law, '#333')
        # Plot total force (left + right)
        fn_tot = [l + r for l, r in zip(d['fn_left'], d['fn_right'])]
        axes[0].plot(d['t'], fn_tot, label=f'{law.upper()} Total GRF', color=c, lw=1.5)
        # Also plot individual for one of them (e.g. SMC) to avoid clutter, or just total is fine
    axes[0].set_title('Ground Reaction Forces (GRF)', fontsize=12, fontweight='bold')
    axes[0].set_ylabel('Force [N]', fontsize=10)
    axes[0].grid(True, linestyle='--', alpha=0.5)
    axes[0].legend(loc='upper right', fontsize=9)
    
    # 2. VMC Joint Action (Knee and Hip)
    for law in ('mpc', 'smc'):
        if law not in data:
            continue
        d = data[law]
        if 'smc' in data and law != 'smc':
            continue # just plot one to avoid clutter, preference to SMC
        axes[1].plot(d['t'], d['knee_pos'], label=f'{law.upper()} Knee Pos (Left)', color='#3498db', lw=1.5)
        axes[1].plot(d['t'], d['hip_pos'], label=f'{law.upper()} Hip Pos (Left)', color='#e74c3c', lw=1.5)
        
    axes[1].set_title('Leg Joint Kinematics (VMC Action)', fontsize=12, fontweight='bold')
    axes[1].set_xlabel('Time [s]', fontsize=10)
    axes[1].set_ylabel('Joint Angle [rad]', fontsize=10)
    axes[1].grid(True, linestyle='--', alpha=0.5)
    axes[1].legend(loc='upper right', fontsize=9)

    plt.tight_layout()
    path = os.path.join(out_dir, 'grf_and_joints.png')
    plt.savefig(path, dpi=300)
    plt.close()
    return path

def plot_smc_sliding_surface(data, out_dir):
    """Layout C: Evolution of the sliding mode variable s1 and phase plane reaching phase."""
    if 'smc' not in data:
        return None
    d = data['smc']
    s1 = d['s1']
    if all(math.isnan(v) for v in s1):
        return None

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5))

    # 1. s1(t)
    ax1.plot(d['t'], s1, color='#2ca02c', lw=1.3, label=r'$s_1(t) = \dot{\theta} + c_1(\theta - \theta^*)$')
    ax1.axhline(0, color='black', linestyle='--', lw=1.0)
    ax1.set_title('Super-Twisting Sliding Variable $s_1(t)$', fontsize=12, fontweight='bold')
    ax1.set_xlabel('Time [s]', fontsize=10)
    ax1.set_ylabel(r'$s_1$ [rad/s]', fontsize=10)
    ax1.grid(True, linestyle='--', alpha=0.5)
    ax1.legend(loc='upper right', fontsize=9)

    # 2. Phase Portrait with Sliding Line
    ax2.plot(d['th'], d['th_d'], color='#2ca02c', lw=1.2, alpha=0.85, label='SMC Trajectory')
    c1 = 10.0
    xlim = ax2.get_xlim()
    th_line = [xlim[0], xlim[1]]
    thd_line = [-c1 * val for val in th_line]
    ax2.plot(th_line, thd_line, color='#8e44ad', linestyle='-.', lw=1.4,
             label=r'Sliding Manifold $s_1 = 0$')
    ax2.axhline(0, color='gray', linestyle=':', lw=0.8)
    ax2.axvline(0, color='gray', linestyle=':', lw=0.8)
    ax2.set_title(r'SMC State Trajectory & Sliding Line in $(\theta, \dot{\theta})$', fontsize=12, fontweight='bold')
    ax2.set_xlabel(r'Pitch $\theta$ [deg]', fontsize=10)
    ax2.set_ylabel(r'Pitch Rate $\dot{\theta}$ [deg/s]', fontsize=10)
    ax2.grid(True, linestyle='--', alpha=0.5)
    ax2.legend(loc='upper right', fontsize=9)

    plt.tight_layout()
    path = os.path.join(out_dir, 'smc_sliding_surface.png')
    plt.savefig(path, dpi=300)
    plt.close()
    return path




def plot_torque_distribution(data, out_dir):
    """Layout E: Hip vs Knee Torque Distribution (TMECH style)."""
    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    
    for law in ('mpc', 'smc'):
        if law not in data: continue
        if law == 'mpc' and 'smc' in data: continue # Avoid clutter, plot SMC
        
        d = data[law]
        axes[0].plot(d['t'], d['hip_u'], label=r'Hip Torque $	au_{hip}$', color='#e74c3c', lw=1.5)
        axes[0].plot(d['t'], d['knee_u'], label=r'Knee Torque $	au_{knee}$', color='#3498db', lw=1.5)
        
        # Calculate ratio, avoid div by zero
        ratio = [abs(h) / (abs(k) + 1e-3) for h, k in zip(d['hip_u'], d['knee_u'])]
        # filter spikes for better visualization
        ratio = [min(r, 10.0) for r in ratio]
        axes[1].plot(d['t'], ratio, label=r'Torque Ratio $|	au_{hip}| / |	au_{knee}|$', color='#9b59b6', lw=1.5)
        
    axes[0].set_title('Torque Distribution (Hip vs Knee)', fontsize=12, fontweight='bold')
    axes[0].set_ylabel('Torque [Nm]', fontsize=10)
    axes[0].grid(True, linestyle='--', alpha=0.5)
    axes[0].legend(loc='upper right')
    
    axes[1].set_title('Joint Torque Ratio', fontsize=12, fontweight='bold')
    axes[1].set_xlabel('Time [s]', fontsize=10)
    axes[1].set_ylabel('Ratio', fontsize=10)
    axes[1].grid(True, linestyle='--', alpha=0.5)
    axes[1].legend(loc='upper right')

    plt.tight_layout()
    path = os.path.join(out_dir, 'torque_distribution.png')
    plt.savefig(path, dpi=300)
    plt.close()
    return path

def plot_3d_attitude(data, out_dir):
    """Layout F: 3D Attitude (Roll & Yaw) for disturbance rejection."""
    fig, axes = plt.subplots(2, 1, figsize=(10, 8), sharex=True)
    
    for law in ('mpc', 'smc'):
        if law not in data: continue
        d = data[law]
        c = COLORS.get(law, '#333')
        
        axes[0].plot(d['t'], d['roll'], label=f'{law.upper()} Roll $\phi$', color=c, lw=1.5)
        axes[1].plot(d['t'], d['yaw'], label=f'{law.upper()} Yaw $\psi$', color=c, lw=1.5)
        
    axes[0].set_title('Roll Attitude (Lateral Disturbance Rejection)', fontsize=12, fontweight='bold')
    axes[0].set_ylabel('Roll [deg]', fontsize=10)
    axes[0].grid(True, linestyle='--', alpha=0.5)
    axes[0].legend(loc='upper right')
    
    axes[1].set_title('Yaw Attitude (Heading Tracking)', fontsize=12, fontweight='bold')
    axes[1].set_xlabel('Time [s]', fontsize=10)
    axes[1].set_ylabel('Yaw [deg]', fontsize=10)
    axes[1].grid(True, linestyle='--', alpha=0.5)
    axes[1].legend(loc='upper right')

    plt.tight_layout()
    path = os.path.join(out_dir, 'attitude_3d.png')
    plt.savefig(path, dpi=300)
    plt.close()
    return path

def generate_plots(run_dir, out_dir=None):
    data = load_run_data(run_dir)
    if not data:
        print(f'No valid CSV data found in {run_dir}')
        return False

    out_dir = out_dir or os.path.join(run_dir, 'plots')
    os.makedirs(out_dir, exist_ok=True)

    # 1. Homework 4 Style: Tracking and Errors Multi-panel
    p1 = plot_tracking_and_errors(data, out_dir)

    # 2. Homework 4 Style: Disturbance Analysis and Phase Portrait
    p2 = plot_disturbance_and_phase_portrait(data, out_dir)

    # 3. SMC Sliding Surface & Phase Dynamics
    p3 = plot_smc_sliding_surface(data, out_dir)
    p4 = plot_ground_reaction_forces_and_joints(data, out_dir)

    print(f'Grafici salvati con successo in: {out_dir}')
    print(f'  - {p1}')
    print(f'  - {p2}')
    if p3:
        print(f'  - {p3}')
    if p4:
        print(f'  - {p4}')
    return True


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('run_dir', help='campaign directory produced by rotino_benchmark campaign')
    p.add_argument('--out', default=None, help='optional output directory for images')
    args = p.parse_args([a for a in (argv if argv is not None else sys.argv[1:]) if a != '--'])

    if not os.path.isdir(args.run_dir):
        p.error(f'Directory non trovata: {args.run_dir}')

    success = generate_plots(args.run_dir, args.out)
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
