"""The benchmark scenarios: one place that says what is run, for how long, and what to look at.

Every scenario is run with the same world, URDF and launch arguments for the PID and the MPC; only the
controller package changes. `key` lists the metrics (compare.METRICS) that summarise the scenario.
"""

from collections import OrderedDict
from dataclasses import dataclass, field


@dataclass
class Scenario:
    title: str
    description: str
    args: list = field(default_factory=list)   # launch arguments of robot.launch.py
    duration: float = 20.0                      # s of logging after the logger starts
    key: tuple = ()


SCENARIOS = OrderedDict([
    ('equilibrio', Scenario(
        'Equilibrio da fermo', 'Rilascio dall\'ancora e equilibrio sul posto.',
        [], 15.0, ('pitch_rms_deg', 'pos_err_rms_m', 'wheel_tau_rms_Nm', 'chatter_Nm'))),
    ('spinta', Scenario(
        'Spinta sul torso', 'Impulso di 2,7 N s all\'indietro sul torso a 4 s dal rilascio.',
        ['push_enable:=true'], 20.0,
        ('pitch_peak_deg', 'recovery_s', 'pos_err_max_m', 'dist_energy_J', 'wheel_load_min_N'))),
    ('trapezio', Scenario(
        'Trapezio di velocita', '2 m in avanti con profilo trapezoidale: 1 m/s, 0,6 m/s^2.',
        ['velocity_enable:=true', 'velocity_max:=1.0'], 20.0,
        ('pos_err_rms_m', 'vel_err_rms_ms', 'pitch_peak_deg', 'recovery_s'))),
    ('va_e_vieni', Scenario(
        'Va e vieni', '1 m avanti e indietro in 8 s (riferimento sinusoidale).',
        ['drive_enable:=true'], 20.0,
        ('pos_err_rms_m', 'vel_err_rms_ms', 'pitch_peak_deg', 'wheel_tau_rms_Nm'))),
    ('altezza', Scenario(
        'Variazione di altezza', 'Altezza sinusoidale +-3 cm, periodo 2,2 s, da fermo.',
        ['height_enable:=true'], 15.0,
        ('pitch_rms_deg', 'pos_err_rms_m', 'height_rms_mm', 'wheel_tau_rms_Nm'))),
    ('curva_S', Scenario(
        'Curva a S', 'S di 3 m con scarto laterale di 0,6 m in 12 s.',
        ['planar_enable:=true'], 25.0,
        ('pos_err_rms_m', 'pitch_peak_deg', 'zmp_lat_ratio_pct', 'wheel_load_min_N'))),
    ('curva_S_veloce', Scenario(
        'Curva a S veloce', 'S di 3 m con scarto laterale di 1 m in 5 s.',
        ['planar_enable:=true', 'traj_duration:=5.0', 'traj_lateral:=1.0'], 15.0,
        ('pos_err_rms_m', 'pitch_peak_deg', 'zmp_lat_ratio_pct', 'wheel_load_min_N'))),
    ('salto', Scenario(
        'Salto', 'Salto verticale a 4 s dal rilascio, poi equilibrio.',
        ['jump_enable:=true'], 15.0,
        ('com_rise_mm', 'pitch_peak_deg', 'recovery_s', 'pos_err_max_m'))),
])
