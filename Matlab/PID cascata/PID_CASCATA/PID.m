% =========================================================================
% SIMULAZIONE COMPLETA WIP: EQUILIBRIO STATICO + CONTROLLO PID
% Integra il modello dinamico simbolico con l'approccio robusto del paper
% =========================================================================
clear; clc; close all;

%% 1. DEFINIZIONE PARAMETRI FISICI
disp('1. Inizializzazione parametri fisici...');
p_mw = 3.5; p_Iw = 0.1; p_Rw = 0.127;
p_m1 = 1.2; p_I1 = 0.0203; p_l1 = 0.45; p_lc1 = p_l1/2;
p_m2 = 5.3; p_I2 = 0.0894; p_l2 = 0.45; p_lc2 = p_l2/2;
p_m3 = 60.0; p_I3 = 0.6125; p_l3 = 0.35; p_lc3 = p_l3/2;
p_g = 9.81;

p.mw = 3.5; p.Iw = 0.1; p.Rw = 0.127;
p.m1 = 1.2; p.I1 = 0.0203; p.l1 = 0.45; p.lc1 = p_l1/2;
p.m2 = 5.3; p.I2 = 0.0894; p.l2 = 0.45; p.lc2 = p_l2/2;
p.m3 = 60.0; p.I3 = 0.6125; p.l3 = 0.35; p.lc3 = p_l3/2;
p.g = 9.81;


%% 2. FASE 1: RICERCA DEL TARGET CINEMATICO (Equilibrio Statico)
disp('2. Calcolo della postura di equilibrio (fsolve)...');
h_desiderata = 0.75; % Altezza desiderata per il bacino [m]

% Sistema di 3 equazioni dal paper: [X_CoM = 0; Postura = pi/2; Z_end = h]
cinematica_vincoli = @(q_var) [
    (p_m1*p_lc1 + (p_m2+p_m3)*p_l1)*cos(q_var(1)) + ...
    (p_m2*p_lc2 + p_m3*p_l2)*cos(q_var(1)+q_var(2)) + ...
    (p_m3*p_lc3)*cos(q_var(1)+q_var(2)+q_var(3)); 
    q_var(1) + q_var(2) + q_var(3) - (pi/2);
    p_l1*sin(q_var(1)) + p_l2*sin(q_var(1)+q_var(2)) + p_l3*sin(q_var(1)+q_var(2)+q_var(3)) - h_desiderata
];

options = optimoptions('fsolve', 'Display', 'off');
q_target = fsolve(cinematica_vincoli, [pi/3; -pi/6; pi/3], options);

fprintf('Target Trovati: q1 = %.2f rad, q2 = %.2f rad, q3 = %.2f rad\n', q_target(1), q_target(2), q_target(3));

%% 3. FASE 2: COSTRUZIONE MODELLO NUMERICO
disp('3. Derivazione modello dinamico simbolico e conversione numerica...');
syms Rw mw m1 m2 m3 l1 l2 l3 g Iw I1 I2 I3 real
syms qw q1 q2 q3 real           
syms dqw dq1 dq2 dq3 real       
syms ddqw ddq1 ddq2 ddq3 real   

Q = [q1; qw; q2; q3]; dQ = [dq1; dqw; dq2; dq3]; ddQ = [ddq1; ddqw; ddq2; ddq3];

% Cinematica dei baricentri
sw = Rw*qw;
x1 = l1/2*cos(q1) + sw;                                y1 = l1/2*sin(q1);
x2 = l1*cos(q1) + l2/2*cos(q1 + q2) + sw;              y2 = l1*sin(q1) + l2/2*sin(q1 + q2);
x3 = l1*cos(q1) + l2*cos(q1 + q2) + l3/2*sin(q3) + sw; y3 = l1*sin(q1) + l2*sin(q1 + q2) + l3/2*cos(q3);

v1 = [jacobian(x1, Q)*dQ; jacobian(y1, Q)*dQ];
v2 = [jacobian(x2, Q)*dQ; jacobian(y2, Q)*dQ];
v3 = [jacobian(x3, Q)*dQ; jacobian(y3, Q)*dQ];

omega_w = dqw; omega_1 = dq1; omega_2 = dq1 + dq2; omega_3 = dq3;

% Energie
Tw = 0.5 * mw * (Rw*dqw)^2 + 0.5 * Iw * omega_w^2;
T1 = 0.5 * m1 * (v1'*v1) + 0.5 * I1 * omega_1^2;
T2 = 0.5 * m2 * (v2'*v2) + 0.5 * I2 * omega_2^2;
T3 = 0.5 * m3 * (v3'*v3) + 0.5 * I3 * omega_3^2;
T = simplify(Tw + T1 + T2 + T3);
U = simplify(mw*g*Rw + m1*g*(Rw+y1) + m2*g*(Rw+y2) + m3*g*(Rw+y3));

% Equazioni di Moto
L = T - U;
Eq_moto = simplify(jacobian(jacobian(L, dQ)', Q)*dQ + jacobian(jacobian(L, dQ)', dQ)*ddQ - jacobian(L, Q)');

% Estrazione Matrici
G_matrix = simplify(subs(Eq_moto, [dQ; ddQ], zeros(8,1)));
M_matrix = simplify(jacobian(Eq_moto, ddQ));
C_matrix_times_dQ = simplify(Eq_moto - M_matrix*ddQ - G_matrix); 

% Sostituzione Numerica
params_sym = [Rw, mw, m1, m2, m3, l1, l2, l3, g, Iw, I1, I2, I3];
params_num = [p_Rw, p_mw, p_m1, p_m2, p_m3, p_l1, p_l2, p_l3, p_g, p_Iw, p_I1, p_I2, p_I3];

M_fun = matlabFunction(subs(M_matrix, params_sym, params_num), 'Vars', {Q});
CdQ_fun = matlabFunction(subs(C_matrix_times_dQ, params_sym, params_num), 'Vars', {Q, dQ});
G_fun = matlabFunction(subs(G_matrix, params_sym, params_num), 'Vars', {Q});

%% 4. FASE 3: SIMULAZIONE ODE45 CON IL CONTROLLORE PID
disp('4. Esecuzione Simulazione ODE45...');
T_end = 30.0; 

% Condizione Iniziale PERTURBATA: 
% Iniziamo con un errore di 0.1 rad (circa 6 gradi) in avanti rispetto al target.
% Il robot dovrà accelerare la ruota per recuperare l'equilibrio.
X0 = [q_target(1) - 0.1; 0; q_target(2); q_target(3); 0; 0; 0; 0]; 

[T_sim, X_sim] = ode45(@(t, X) robot_dynamics(t, X, M_fun, CdQ_fun, G_fun, q_target), [0 T_end], X0);

%% VIDEO
%% Per il Video 
out.q.Data=[X_sim(:,2), X_sim(:,1), X_sim(:,3), X_sim(:,4)];
out.tout = T_sim;
robot_params=p;

%% 5. PLOTTING
figure('Name', 'Stabilizzazione WIP tramite PID Decentralizzato', 'Position', [100 100 900 600]);

subplot(2,2,1);
plot(T_sim, X_sim(:, 1), 'b', 'LineWidth', 2); hold on;
yline(q_target(1), 'r--', 'LineWidth', 1.5);
title('Stinco (q1) - Passivo'); ylabel('[rad]'); grid on;
legend('Stato', 'Target');

subplot(2,2,2);
plot(T_sim, X_sim(:, 2), 'b', 'LineWidth', 2);
title('Ruota (qw) - Attivo'); ylabel('[rad]'); grid on;
legend('Posizione ruota');

subplot(2,2,3);
plot(T_sim, X_sim(:, 3), 'b', 'LineWidth', 2); hold on;
yline(q_target(2), 'r--', 'LineWidth', 1.5);
title('Ginocchio (q2) - Attivo'); xlabel('Tempo [s]'); ylabel('[rad]'); grid on;

subplot(2,2,4);
plot(T_sim, X_sim(:, 4), 'b', 'LineWidth', 2); hold on;
yline(q_target(3), 'r--', 'LineWidth', 1.5);
title('Torso (q3) - Attivo'); xlabel('Tempo [s]'); ylabel('[rad]'); grid on;

disp('Simulazione completata con successo!');

% =========================================================================
% FUNZIONE DINAMICA DEL ROBOT
% =========================================================================
function dX = robot_dynamics(t, X, M_fun, CdQ_fun, G_fun, q_target)
    % Estraggo lo stato
    q = X(1:4); % [q1; qw; q2; q3]
    dq = X(5:8);
    
    M = M_fun(q);
    CdQ = CdQ_fun(q, dq);
    G = G_fun(q);
    
    %% --- LA "STRAIGHTFORWARD IDEA" DEL PAPER ---
    
    % 1. La Ruota salva il robot bilanciando lo stinco (q1)
    Kp_w = 400;  % Guadagno proporzionale ruota
    Kd_w = 80;   % Guadagno derivativo ruota
    
    e1 = q_target(1) - q(1);
    de1 = 0 - dq(1);
    % Se e1 > 0 (robot cade in avanti), tau_w > 0 spinge la ruota in avanti per recuperare.
    tau_w = Kp_w * e1 + Kd_w * de1; 
    
    % 2. Le articolazioni si fissano rigidamente sui target (q2, q3)
    Kp_j = 800; 
    Kd_j = 50;  
    
    e2 = q_target(2) - q(3);
    de2 = 0 - dq(3);
    tau_2 = Kp_j * e2 + Kd_j * de2;
    
    e3 = q_target(3) - q(4);
    de3 = 0 - dq(4);
    tau_3 = Kp_j * e3 + Kd_j * de3;
    
    %% Integrazione
    % Vettore forze: nessuna coppia su q1(indice 1), motori su qw(2), q2(3), q3(4)
    tau_full = [0; tau_w; tau_2; tau_3]; 
    
    ddQ = M \ (tau_full - CdQ - G);
    dX = [dq; ddQ];
end