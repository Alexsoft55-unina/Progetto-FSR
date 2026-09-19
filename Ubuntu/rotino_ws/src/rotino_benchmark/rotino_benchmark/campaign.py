"""Runs one scenario against several control laws, headless, and collects a CSV per law.

Every run uses the same world, the same URDF and the same scenario arguments; only the controller
package changes. Runs are sequential and each one starts from a clean process table, because a
leftover node from a previous run keeps publishing on /rotino/* and silently corrupts the next
result (a stale dashboard publishing /rotino/cmd_vel put a controller into teleop once).

    ros2 run rotino_benchmark campaign -- --scenario push_enable:=true --duration 25
"""

import argparse
import os
import signal
import subprocess
import sys
import time
from datetime import datetime

CONTROLLERS = ('pid', 'mpc', 'smc')
STALE_PATTERNS = ('rotino_pid/controller', 'rotino_mpc/controller', 'rotino_smc/controller',
                  'rotino_benchmark/logger', 'rotino_dashboard', 'gz sim', 'parameter_bridge',
                  'robot_state_publisher', 'ros_gz_sim')


def kill_stale():
    """Nothing from a previous run may still be publishing on /rotino/*."""
    for pat in STALE_PATTERNS:
        subprocess.run(['pkill', '-f', pat], capture_output=True)
    time.sleep(2.0)


def run_one(law, scenario, duration, out_dir, log_dir):
    print(f'\n=== {law.upper()} ===', flush=True)
    kill_stale()
    os.makedirs(out_dir, exist_ok=True)

    launch = ['ros2', 'launch', f'rotino_{law}', f'rotino_{law}.launch.py', 'gui:=false', *scenario]
    logger = ['ros2', 'run', 'rotino_benchmark', 'logger', '--ros-args',
              '-p', 'use_sim_time:=true', '-p', f'controller:={law}', '-p', f'output_dir:={out_dir}']

    with open(os.path.join(log_dir, f'{law}.log'), 'w') as sim_log:
        sim = subprocess.Popen(launch, stdout=sim_log, stderr=subprocess.STDOUT,
                               preexec_fn=os.setsid)
        time.sleep(6.0)                       # let Gazebo and the controllers come up
        log = subprocess.Popen(logger, stdout=sim_log, stderr=subprocess.STDOUT,
                               preexec_fn=os.setsid)
        try:
            time.sleep(duration)
        finally:
            for proc in (log, sim):
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                except ProcessLookupError:
                    pass
            time.sleep(3.0)
            for proc in (log, sim):
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
    kill_stale()

    produced = sorted(f for f in os.listdir(out_dir) if f.startswith(f'rotino_{law}_'))
    if not produced:
        print(f'  NO CSV produced for {law} - see {log_dir}/{law}.log', flush=True)
        return None
    print(f'  {produced[-1]}', flush=True)
    return os.path.join(out_dir, produced[-1])


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--scenario', nargs='*', default=[],
                   help='launch arguments, e.g. push_enable:=true velocity_max:=1.0')
    p.add_argument('--controllers', default=','.join(CONTROLLERS),
                   help=f'comma-separated subset of {",".join(CONTROLLERS)}')
    p.add_argument('--duration', type=float, default=25.0, help='seconds of logging per run')
    p.add_argument('--name', default=None, help='name of the run directory')
    p.add_argument('--out', default=os.path.expanduser('~/rotino_ws/benchmark_runs'))
    args = p.parse_args([a for a in (argv if argv is not None else sys.argv[1:]) if a != '--'])

    laws = [c.strip() for c in args.controllers.split(',') if c.strip()]
    unknown = [c for c in laws if c not in CONTROLLERS]
    if unknown:
        p.error(f'unknown controller(s): {", ".join(unknown)}')

    tag = args.name or (('_'.join(a.split(':=')[0] for a in args.scenario) or 'balance')
                        + f'_{datetime.now():%Y%m%d_%H%M%S}')
    run_dir = os.path.join(args.out, tag)
    log_dir = os.path.join(run_dir, 'logs')
    os.makedirs(log_dir, exist_ok=True)

    print(f'scenario : {" ".join(args.scenario) or "(balance only)"}')
    print(f'laws     : {", ".join(laws)}')
    print(f'duration : {args.duration:.0f} s each')
    print(f'output   : {run_dir}')

    for law in laws:
        run_one(law, args.scenario, args.duration, run_dir, log_dir)

    print(f'\nDone. Compare with:\n  ros2 run rotino_benchmark compare -- {run_dir}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
