"""Offline checks of the PID vs MPC comparison (compare, scenarios): no simulation, synthetic logs."""

import csv
import json
import math
import os

import pytest

from rotino_benchmark import compare
from rotino_benchmark.common import LAWS, find_csv, is_suite, scenario_dirs
from rotino_benchmark.scenarios import SCENARIOS

HEADER = ['time_s', 'theta_deg', 'x_m', 'xdot_ms', 'wheel_u', 'com_z_m', 'fn_total_N', 'loaded',
          'wheel_L_torque_cmd', 'wheel_R_torque_cmd', 'theta_err_deg', 's_err_m', 'xdot_err_ms', 'push_force_N']


def write_log(path, pitch_amp, pos_err, n=4000):
    """A damped pitch oscillation and a constant position error: easy to predict metrics."""
    with open(path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(HEADER)
        for k in range(n):
            t = 0.002 * k
            th = pitch_amp * math.exp(-(t - 2.0)) * math.cos(6.0 * (t - 2.0)) if t >= 2.0 else 0.0
            w.writerow([t, th, 0.1 * t, 0.1, 0.0, 0.19, 42.0, 1, 0.5, 0.5, th, pos_err, 0.0, 0.0])


def test_catalogue_uses_known_metrics_and_arguments():
    for name, sc in SCENARIOS.items():
        assert sc.key, name
        assert all(k in compare.METRIC for k in sc.key), name
        assert all(':=' in a for a in sc.args), name
        assert sc.duration >= 10.0


def test_better_follows_the_direction_of_the_metric():
    assert compare.better('pitch_peak_deg', {'pid': 5.0, 'mpc': 8.0}) == 'pid'        # lower is better
    assert compare.better('wheel_load_min_N', {'pid': 5.0, 'mpc': 8.0}) == 'mpc'      # higher is better
    assert compare.better('pitch_peak_deg', {'pid': 5.0, 'mpc': 5.1}) == '='          # within 5 %
    assert compare.better('travel_m', {'pid': 1.0, 'mpc': 3.0}) == ''                 # descriptive
    assert compare.better('pitch_peak_deg', {'pid': 5.0, 'mpc': float('nan')}) == ''
    assert compare.better('pitch_rms_deg', {'pid': 0.001, 'mpc': 0.0}) == '='         # under the resolution


def test_metrics_of_a_synthetic_log(tmp_path):
    p = tmp_path / 'rotino_pid_1.csv'
    write_log(p, 4.0, 0.02)
    m = compare.metrics(compare.read_csv(str(p)))
    assert math.isclose(m['pitch_peak_deg'], 4.0, rel_tol=1e-6)
    assert math.isclose(m['pos_err_rms_m'], 0.02, rel_tol=1e-6)
    assert math.isclose(m['pos_err_max_m'], 0.02, rel_tol=1e-6)
    assert math.isclose(m['wheel_tau_rms_Nm'], 0.5, rel_tol=1e-6)
    assert m['chatter_Nm'] == 0.0 and m['airborne_pct'] == 0.0
    assert 0.0 < m['recovery_s'] < 6.0


def test_release_transient_is_not_the_peak(tmp_path):
    p = tmp_path / 'rotino_pid_1.csv'
    with open(p, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(HEADER)
        for k in range(4000):
            t = 0.002 * k
            th = 4.7 * math.exp(-5 * t) + (1.0 if 3.0 < t < 3.5 else 0.0)   # release, then a 1 deg event
            w.writerow([t, th, 0, 0, 0, 0.19, 42, 1, 0, 0, th, 0, 0, 0])
    assert math.isclose(compare.metrics(compare.read_csv(str(p)))['pitch_peak_deg'], 1.0, rel_tol=1e-3)


def make_suite(root):
    for name, (pid, mpc) in (('spinta', (6.0, 9.0)), ('trapezio', (3.0, 2.0))):
        d = root / name
        d.mkdir()
        write_log(d / 'rotino_pid_1.csv', pid, 0.01)
        write_log(d / 'rotino_mpc_1.csv', mpc, 0.03)
        (d / 'scenario.json').write_text(json.dumps({'name': name}))
    return root


def test_suite_report_counts_the_wins(tmp_path, monkeypatch):
    monkeypatch.setattr(compare, 'zmp_row_metrics', lambda path, result=None: {})   # no URDF needed
    suite = make_suite(tmp_path)
    assert is_suite(str(suite)) and len(scenario_dirs(str(suite))) == 2
    results = compare.analyse_suite(str(suite), plots=False)
    names = [compare.scenario_name(d) for d, _ in results]
    assert names == ['spinta', 'trapezio']                      # catalogue order, not alphabetical
    text = (suite / 'riepilogo.md').read_text()
    assert 'Spinta sul torso' in text and 'Trapezio' in text
    rows = list(csv.DictReader(open(suite / 'riepilogo.csv')))
    peak = {r['scenario']: r['migliore'] for r in rows if r['metrica'] == 'pitch_peak_deg'}
    assert peak == {'spinta': 'pid', 'trapezio': 'mpc'}
    for d in scenario_dirs(str(suite)):
        assert os.path.exists(os.path.join(d, 'confronto.md'))
        assert os.path.exists(os.path.join(d, 'metriche.json'))


def test_cached_metrics_are_reused_until_the_logs_change(tmp_path, monkeypatch):
    calls = []
    monkeypatch.setattr(compare, 'zmp_row_metrics', lambda path, result=None: calls.append(path) or {})
    d = tmp_path / 'spinta'
    d.mkdir()
    write_log(d / 'rotino_pid_1.csv', 5.0, 0.0)
    compare.scenario_metrics(str(d))
    compare.scenario_metrics(str(d))
    assert len(calls) == 1                                       # second call from metriche.json
    write_log(d / 'rotino_mpc_1.csv', 5.0, 0.0)                  # a new log invalidates the cache
    found = compare.scenario_metrics(str(d))
    assert set(found) == set(LAWS) and len(calls) == 3
    assert find_csv(str(d), 'mpc').endswith('rotino_mpc_1.csv')


@pytest.mark.parametrize('name', list(SCENARIOS))
def test_scenario_report_marks_the_key_metrics(tmp_path, monkeypatch, name):
    monkeypatch.setattr(compare, 'zmp_row_metrics', lambda path, result=None: {})
    d = tmp_path / name
    d.mkdir()
    write_log(d / 'rotino_pid_1.csv', 5.0, 0.0)
    write_log(d / 'rotino_mpc_1.csv', 4.0, 0.0)
    found = compare.scenario_metrics(str(d))
    compare.write_scenario_report(str(d), name, found)
    md = (d / 'confronto.md').read_text()
    assert SCENARIOS[name].title in md
    for key in SCENARIOS[name].key:
        assert f'**{compare.METRIC[key][1]}**' in md
