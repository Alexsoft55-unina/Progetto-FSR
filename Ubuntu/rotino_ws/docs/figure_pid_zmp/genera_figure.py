"""Figures of docs/PID_ZMP.md from the benchmark runs: python3 docs/figure_pid_zmp/genera_figure.py (from the workspace root)."""
import csv
import glob
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

RUNS = 'benchmark_runs/archivio_2026-09-30_01'
OUT = 'docs/figure_pid_zmp'
os.makedirs(OUT, exist_ok=True)
COL = {'pid': '#d62728', 'old': '#7f7f7f', 'mpc': '#1f77b4'}
HALF_TRACK_MM = 147.0


def log(run, law):
    path = sorted(glob.glob(f'{RUNS}/{run}/rotino_{law}_*.csv'))[-1]
    rows = list(csv.DictReader(open(path)))
    out = {}
    for k in rows[0]:
        try:
            out[k] = np.array([float(r[k]) if r[k] not in ('', 'nan') else np.nan for r in rows])
        except ValueError:
            pass
    return out


def zmp(run, law, lg):
    """ZMP series of the analysis, on the logger time base (time after release)."""
    rows = list(csv.DictReader(open(f'{RUNS}/{run}/plots/zmp_{law}.csv')))
    z = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]}
    ok = np.isfinite(lg['odom_stamp_s'])
    z['t'] = z['t_s'] - np.median(lg['odom_stamp_s'][ok] - lg['time_s'][ok])
    return z


# ---------------------------------------------------------------- 1. trapezoid: ZMP preview
lp = log('cmp_zmp_velocity', 'pid')
t = lp['time_s']
fig, ax = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
ax[0].plot(t, lp['xdot_ref_ms'], 'k--', lw=1.2, label='riferimento')
for law in ('pid', 'mpc'):
    lg = lp if law == 'pid' else log('cmp_zmp_velocity', law)
    ax[0].plot(lg['time_s'], lg['xdot_ms'], color=COL[law], lw=1.1, label=law.upper())
    ax[2].plot(lg['time_s'], lg['theta_deg'], color=COL[law], lw=1.1, label=law.upper())
ax[0].set_ylabel('velocità [m/s]')
ax[0].legend(ncol=3, fontsize=8)
ax[1].plot(t, lp['zmp_des_mm'], color='k', lw=1.1, ls='--', label='ZMP desiderato (PID)')
ax[1].plot(t, lp['zmp_ctrl_mm'], color=COL['pid'], lw=1.1, label='ZMP = contatto (PID)')
ax[1].plot(t, -1e3 * 0.193 / 9.81 * lp['acc_ref_ms2'], color='0.6', lw=1.0, label='-h a_ref / g (anteprima LIPM)')
ax[1].set_ylabel('ZMP davanti al CoM [mm]')
ax[1].legend(fontsize=8)
ax[2].set_ylabel('inclinazione CoM [deg]')
ax[2].set_xlabel('tempo dal rilascio [s]')
ax[2].legend(ncol=2, fontsize=8)
ax[0].set_xlim(1.0, 9.0)
for a in ax:
    a.grid(alpha=0.3)
fig.suptitle('Trapezio 1 m/s: il PID-ZMP sposta lo ZMP dietro il CoM prima di accelerare')
fig.tight_layout()
fig.savefig(f'{OUT}/trapezio_zmp.png', dpi=130)

# ---------------------------------------------------------------- 2. fast S: before / after
fig, ax = plt.subplots(2, 1, figsize=(9, 6.5), sharex=True)
for run, law, label, c in (('zmp_planar_fast', 'pid', 'PID precedente', COL['old']),
                           ('cmp_zmp_planar_fast', 'pid', 'PID-ZMP', COL['pid']),
                           ('cmp_zmp_planar_fast', 'mpc', 'MPC', COL['mpc'])):
    lg = log(run, law)
    z = zmp(run, law, lg)
    ax[0].plot(z['t'], 100 * z['zmp_lat_mm'] / HALF_TRACK_MM, color=c, lw=1.0, label=label)
    tau = 0.5 * (lg['wheel_L_torque_cmd'] + lg['wheel_R_torque_cmd'])
    ax[1].plot(lg['time_s'], tau, color=c, lw=0.8, label=label)
ax[0].axhspan(100, 160, color='0.9')
ax[0].axhspan(-160, -100, color='0.9')
ax[0].set_ylim(-160, 160)
ax[0].set_ylabel('ZMP laterale / (d/2) [%]')
ax[0].legend(ncol=3, fontsize=8)
ax[1].set_ylabel('coppia ruote media [Nm]')
ax[1].set_xlabel('tempo dal rilascio [s]')
ax[1].set_xlim(0.0, 12.0)
for a in ax:
    a.grid(alpha=0.3)
fig.suptitle('Curva a S veloce: fuori dall\'appoggio (zona grigia) il PID precedente, dentro il PID-ZMP')
fig.tight_layout()
fig.savefig(f'{OUT}/curva_S_veloce_prima_dopo.png', dpi=130)

# ---------------------------------------------------------------- 3. lateral lean experiment
fig, ax = plt.subplots(3, 1, figsize=(9, 7.5), sharex=True)
for run, label, c in (('lat_check_off', 'gambe pari (default)', COL['pid']),
                      ('lat_check', 'inclinazione in curva (zmp_lateral)', '#9467bd')):
    lg = log(run, 'pid')
    z = zmp(run, 'pid', lg)
    ax[0].plot(z['t'], z['zmp_lat_mm'], color=c, lw=1.0, label=label)
    ax[1].plot(lg['time_s'], lg['roll_deg'], color=c, lw=1.0, label=label)
    ax[2].plot(lg['time_s'], lg['base_wz'], color=c, lw=1.0, label=label)
ax[0].set_ylabel('ZMP laterale [mm]')
ax[0].legend(fontsize=8)
ax[1].set_ylabel('rollio torso [deg]')
ax[2].set_ylabel('imbardata [rad/s]')
ax[2].set_xlabel('tempo dal rilascio [s]')
ax[2].set_xlim(2.0, 7.5)
ax[2].axvspan(4.5, 4.9, color='0.9')
for a in ax:
    a.grid(alpha=0.3)
fig.suptitle('Inclinazione in curva: al flesso il camber si inverte e la ruota cilindrica cambia spigolo')
fig.tight_layout()
fig.savefig(f'{OUT}/inclinazione_laterale.png', dpi=130)
print('ok', sorted(os.listdir(OUT)))
