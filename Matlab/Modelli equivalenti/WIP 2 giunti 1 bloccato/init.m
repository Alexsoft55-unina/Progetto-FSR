% =========================================================================
% Script di Inizializzazione — Modello Lagrangiano Bipede 2-Link
% Convenzione coordinate generalizzate:
%   Q = [qw; q1; q2]
%   qw  = rotazione ruota (rad)
%   q1  = angolo stinco rispetto alla verticale (rad)
%   q2  = angolo coscia rispetto alla verticale (rad)
%   meq = massa concentrata all'estremo del link 2 (anca/corpo)
% =========================================================================
clc; clear;

%% 1. Parametri fisici numerici
robot_params.mw  = 3.5;        % [kg]   massa ruota
robot_params.Rw  = 0.127;      % [m]   raggio ruota
robot_params.Iw  = robot_params.mw *robot_params.Rw.^2;        % [kg m^2] inerzia ruota
robot_params.m1  = 1.2;        % [kg]   massa stinco
robot_params.m2  = 5.3;        % [kg]   massa coscia
robot_params.meq = 60.0;15;       % [kg]   massa concentrata estremo link2
robot_params.l1  = 0.45;       % [m]   lunghezza stinco
robot_params.l2  = 0.45;       % [m]   lunghezza coscia
robot_params.lc1 = robot_params.l1 / 2;  % [m] distanza CoM stinco da giunto prossimale
robot_params.lc2 = robot_params.l2 / 2;  % [m] distanza CoM coscia da ginocchio
robot_params.g   = 9.81;       % [m/s^2]
robot_params.I1  = (1/12) * robot_params.m1 * robot_params.l1^2;  % [kg m^2]
robot_params.I2  = (1/12) * robot_params.m2 * robot_params.l2^2;  % [kg m^2]
robot_params.b1 = 0.1e3;
robot_params.bw = 0.1e1;
robot_params.b2 = .1e2;


Fast_Ts = 1e-4;

K1 = robot_params.m1  * robot_params.lc1 + (robot_params.m2 + robot_params.meq)*robot_params.l1;
K2 = robot_params.m2  * robot_params.lc2 + robot_params.meq*robot_params.l2;

q1_init =0.01; -pi/42;0.01;
q2_init = asin(-K1/K2*sin(q1_init));
% 
% gains = [ 1600;                 % kp1     -> omega_n = 4 rad/s, zeta = 1
%            800;                 % kd1
%          144*1.5;                 % kp_pc   -> omega_n = 12 rad/s, zeta = 1
%           24;                 % kd_pc
%         0.15;                 % kv      -> 1 m/s di errore = 5 cm di offset CoM
%         0.1;                 % pc_max  -> 5 cm (era 4 mm: rail permanente)
%          100;                 % tau_max -> alza per il debug, poi stringi
%          q1_init];

gains = [ 1600;        % kp1     -> omega_n = 4 rad/s
           800;        % kd1     -> zeta = 1
         144;        % kp_pc   -> omega_n = 12 rad/s (3x kp1)
          24;        % kd_pc   -> zeta = 1
        0.025;       % kv      -> T_v ≈ 4 s
        0.0025;      % kx      -> kv/10
        0.12;        % theta_max [rad] ≈ 7°
        0.05;        % pc_max [m]
          40 ];       % tau_max [Nm]