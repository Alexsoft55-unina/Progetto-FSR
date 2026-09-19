"""Turns the CSVs of one campaign into a side-by-side table of the control laws.

Reads only columns every RoTino controller publishes on /rotino/debug, so PID, MPC and SMC are
scored on exactly the same signals.

    ros2 run rotino_benchmark compare -- ~/rotino_ws/benchmark_runs/<run>
"""

import argparse
import csv
import glob
import math
import os
import sys

SETTLE_BAND_DEG = 0.5


def read_csv(path):
    with open(path, newline='') as f:
        rows = list(csv.DictReader(f))
    out = {}
    for key in ('time_s', 'theta_deg', 'x_m', 'xdot_ms', 'wheel_u', 'com_z_m',
                'fn_total_N', 'loaded', 'wheel_L_torque_cmd', 'wheel_R_torque_cmd'):
        col = []
        for r in rows:
            try:
                col.append(float(r.get(key, 'nan')))
            except (TypeError, ValueError):
                col.append(float('nan'))
        out[key] = col
    return out


def finite(xs):
    return [x for x in xs if not math.isnan(x)]


def metrics(d):
    t = d['time_s']
    th = d['theta_deg']
    if not finite(t):
        return None
    dur = max(finite(t)) - min(finite(t))

    # steady state: last third of the run
    cut = min(finite(t)) + 2.0 * dur / 3.0
    tail = [v for ti, v in zip(t, th) if not math.isnan(ti) and ti >= cut and not math.isnan(v)]
    rms = math.sqrt(sum(v * v for v in tail) / len(tail)) if tail else float('nan')

    peak = max((abs(v) for v in finite(th)), default=float('nan'))

    # recovery: last instant the pitch was outside the band, measured from the peak onwards
    settle = float('nan')
    fin = [(ti, v) for ti, v in zip(t, th) if not math.isnan(ti) and not math.isnan(v)]
    if fin:
        i_peak = max(range(len(fin)), key=lambda i: abs(fin[i][1]))
        after = fin[i_peak:]
        out_of_band = [ti for ti, v in after if abs(v) > SETTLE_BAND_DEG]
        if out_of_band:
            settle = max(out_of_band) - fin[i_peak][0]
        elif after:
            settle = 0.0

    taus = [0.5 * (a + b) for a, b in zip(d['wheel_L_torque_cmd'], d['wheel_R_torque_cmd'])
            if not (math.isnan(a) or math.isnan(b))]
    tau_rms = math.sqrt(sum(v * v for v in taus) / len(taus)) if taus else float('nan')
    # chattering proxy: mean |d(tau)| per sample over the steady tail
    tail_tau = taus[int(len(taus) * 0.66):] if taus else []
    chat = (sum(abs(b - a) for a, b in zip(tail_tau, tail_tau[1:])) / max(len(tail_tau) - 1, 1)
            if len(tail_tau) > 2 else float('nan'))

    loaded = finite(d['loaded'])
    airborne = 100.0 * sum(1 for v in loaded if v < 0.5) / len(loaded) if loaded else float('nan')

    x = finite(d['x_m'])
    v = finite(d['xdot_ms'])
    z = finite(d['com_z_m'])
    return {
        'duration_s': dur,
        'pitch_rms_deg': rms,
        'pitch_peak_deg': peak,
        'recovery_s': settle,
        'travel_m': (max(x) - min(x)) if x else float('nan'),
        'speed_peak_ms': max((abs(q) for q in v), default=float('nan')),
        'height_rms_mm': (1000.0 * math.sqrt(sum((q - (sum(z) / len(z))) ** 2 for q in z) / len(z))
                          if z else float('nan')),
        'wheel_tau_rms_Nm': tau_rms,
        'chatter_Nm': chat,
        'airborne_pct': airborne,
    }


ROWS = [
    ('duration_s', 'durata [s]', '{:.1f}'),
    ('pitch_rms_deg', 'beccheggio rms a regime [deg]', '{:.3f}'),
    ('pitch_peak_deg', 'beccheggio di picco [deg]', '{:.2f}'),
    ('recovery_s', 'recupero dopo il picco [s]', '{:.2f}'),
    ('speed_peak_ms', 'velocita di picco [m/s]', '{:.2f}'),
    ('travel_m', 'spazio percorso [m]', '{:.2f}'),
    ('height_rms_mm', 'oscillazione di quota rms [mm]', '{:.1f}'),
    ('wheel_tau_rms_Nm', 'coppia ruote rms [Nm]', '{:.3f}'),
    ('chatter_Nm', 'chattering, d(tau)/campione [Nm]', '{:.4f}'),
    ('airborne_pct', 'tempo senza contatto [%]', '{:.1f}'),
]


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('run_dir', help='campaign directory produced by rotino_benchmark campaign')
    args = p.parse_args([a for a in (argv if argv is not None else sys.argv[1:]) if a != '--'])

    found = {}
    for law in ('pid', 'mpc', 'smc'):
        files = sorted(glob.glob(os.path.join(args.run_dir, f'rotino_{law}_*.csv')))
        if files:
            m = metrics(read_csv(files[-1]))
            if m:
                found[law] = m
    if not found:
        print(f'no CSV found in {args.run_dir}')
        return 1

    laws = list(found)
    width = max(len(lbl) for _, lbl, _ in ROWS) + 2
    print(f'\ncampagna: {os.path.basename(os.path.normpath(args.run_dir))}\n')
    print('  ' + 'metrica'.ljust(width) + ''.join(l.upper().rjust(12) for l in laws))
    print('  ' + '-' * (width + 12 * len(laws)))
    for key, label, fmt in ROWS:
        cells = ''
        for law in laws:
            v = found[law].get(key, float('nan'))
            cells += ('n/d' if math.isnan(v) else fmt.format(v)).rjust(12)
        print('  ' + label.ljust(width) + cells)
    print('\n  Nota: il PID aziona le gambe con un PD di giunto, MPC e SMC con controllo task-space.')
    print('  Stesso URDF e stessa attuazione in coppia, ma la struttura del controllo delle gambe differisce.')
    return 0


if __name__ == '__main__':
    sys.exit(main())
