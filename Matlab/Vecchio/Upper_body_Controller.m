% =========================================================================
% Simulazione Completa: Upper-Body (MPC) + Stance Task-Space (VMC Eq. 19)
% Riferimento: "Modeling and Control of a Wheeled Biped Robot" (MDPI)
% =========================================================================
clear; clc; close all;

%% 1. Parametri del Sistema e Assunzioni
mb = 73;                % Massa dell'upper body [kg] 
g = 9.81;               % Accelerazione di gravità [m/s^2]
dt = 0.01;              % Tempo di campionamento [s] (100 Hz)
N = 15;                 % Orizzonte di predizione

% Limiti fisici
mu = 0.6;               % Coefficiente di attrito 
L_max = 0.8;            % Lunghezza massima della gamba [m]
zb = 0.6;               % Distanza verticale asse-anca [m]
F_min = 100;            % Forza z minima [N]
F_max = 1500;           % Forza z massima [N]

% Stato: x = [s; s_dot; z; z_dot]
% Ingresso: u = [Delta_s; F_z]
dc = [0; 0; 0; -g];     % Disturbo di gravità
dk = dc * dt;

%% 2. Pesi della Funzione Obiettivo e Matrici fisse
S = diag([1000, 10, 1000, 10]); % Pesi alti sulle posizioni
W = diag([1, 0.001]);           % Pesi sugli ingressi

S_bar = 10*kron(eye(N), S);
W_bar = 1/100*kron(eye(N), W);

%% 3. Setup della Simulazione e Generazione Traiettoria
T_sim = 10;                      
steps = round(T_sim / dt);
t_vec = (0:steps-1)*dt;

% Condizioni Iniziali
x0 = [0; 0; 0.6; 0]; 

% Generazione Traiettoria Minimum Jerk
T_manovra = 5.0; % Tempo desiderato per compiere il movimento
x_ref = zeros(4, steps);
for k = 1:steps
    t_curr = t_vec(k);
    if t_curr > 1.0
        [s_des, s_dot_des] = genera_profilo_quinto_ordine(t_curr - 1.0, 1.0, T_manovra);
    else
        s_des = 0; s_dot_des = 0;
    end
    x_ref(1, k) = s_des;
    x_ref(2, k) = s_dot_des;
    x_ref(3, k) = 0.6; % Manteniamo l'altezza target a 0.6m
end

% Logging
x_log = zeros(4, steps);
u_log = zeros(2, steps);
e_log = zeros(4, steps);
J_log = zeros(1, steps); 
x_log(:,1) = x0;

options = optimoptions('quadprog', 'Display', 'off'); 

%% 4. Loop di Simulazione (Linear Time-Varying MPC)
for k = 1:steps-1
    x_k = x_log(:, k);
    h_k = x_k(3); % Altezza attuale z 
    
    % --- Ricalcolo Dinamica Locale basata su h_k ---
    Ac = [0 1 0 0; 0 0 0 0; 0 0 0 1; 0 0 0 0];
    Bc = [0 0; g/h_k 0; 0 0; 0 1/mb];
    Ak = eye(4) + Ac * dt;
    Bk = Bc * dt;
    
    % --- Aggiornamento Dinamico dei Vincoli ---
    max_kinematic_ds = sqrt(L_max^2 - zb^2);
    ds_min = max(-mu*h_k, -max_kinematic_ds);
    ds_max = min(mu*h_k, max_kinematic_ds);
    U_min = [ds_min; F_min];
    U_max = [ds_max; F_max];
    LB = repmat(U_min, N, 1);
    UB = repmat(U_max, N, 1);
    
    % --- Calcolo Matrici di Predizione per l'orizzonte N ---
    Phi = zeros(4*N, 2*N);
    Psi = zeros(4*N, 4);
    Term_dist = zeros(4*N, 1);
    
    for i = 1:N
        Psi((i-1)*4+1:i*4, :) = Ak^i;
        for j = 1:i
            Phi((i-1)*4+1:i*4, (j-1)*2+1:j*2) = (Ak^(i-j)) * Bk;
            Term_dist((i-1)*4+1:i*4) = Term_dist((i-1)*4+1:i*4) + (Ak^(i-j))*dk;
        end
    end
    
    H = 2 * (Phi' * S_bar * Phi + W_bar);
    
    % --- Vettore Riferimento ---
    X_ref_vec = zeros(4*N, 1);
    for i = 1:N
        idx = min(k+i, steps); 
        X_ref_vec((i-1)*4+1:i*4) = x_ref(:, idx);
    end
    
    f = 2 * Phi' * S_bar * (Psi * x_k + Term_dist - X_ref_vec);
    
    % --- Risoluzione QP ---
    [U_seq, fval] = quadprog(H, f, [], [], [], [], LB, UB, [], options);
    
    % --- Calcolo del vero costo (aggiungendo la costante ignorata dal solver) ---
    Errore_non_controllato = (Psi * x_k + Term_dist - X_ref_vec);
    cost_costante = Errore_non_controllato' * S_bar * Errore_non_controllato;
    
    u_opt = U_seq(1:2);
    u_log(:, k) = u_opt;
    J_log(k) = fval + cost_costante;
    e_log(:, k) = x_k - x_ref(:, k);
    
    % Applica al sistema 
    x_log(:, k+1) = Ak * x_k + Bk * u_opt + dk;
end
e_log(:, end) = x_log(:, end) - x_ref(:, end);

%% 5. Stance Task-Space Controller (Calcolo Coppie Giunti - Eq 19)
disp('Calcolo della cinematica inversa e delle coppie dei giunti (Virtual Model Control)...');

l1 = 0.45; % Lunghezza Tibia [m]
l2 = 0.45; % Lunghezza Coscia [m]

% Parametri Virtual Model Control (Eq. 19)
k_p = diag([2000, 2000]); % Matrice di Stiffness virtuale (X, Z) [N/m]
k_d = diag([100, 100]);   % Matrice di Damping virtuale (X, Z) [Ns/m]

tau_hip_log = zeros(1, steps-1);
tau_knee_log = zeros(1, steps-1);
theta1_log = zeros(1, steps-1);
theta2_log = zeros(1, steps-1);

for k = 1:steps-1
    Fz_cmd = u_log(2, k); 
    delta_s = u_log(1, k);
    h_cmd = x_log(3, k); 
    
    % --- 1. Cinematica Inversa (IK) ---
    L_sq = delta_s^2 + h_cmd^2;
    L = sqrt(L_sq);
    
    alpha = atan2(h_cmd, delta_s); 
    cos_beta = (l1^2 + L_sq - l2^2) / (2 * l1 * L);
    cos_beta = max(min(cos_beta, 1), -1); 
    beta = acos(cos_beta);
    
    theta1 = alpha - beta; 
    xk = l1 * cos(theta1);
    yk = l1 * sin(theta1);
    theta2 = atan2(h_cmd - yk, delta_s - xk);
    
    theta1_log(k) = theta1;
    theta2_log(k) = theta2;
    
    % --- 2. Calcolo dello Jacobiano (J) ---
    J11 = -l1 * sin(theta1);
    J12 = -l2 * sin(theta2);
    J21 =  l1 * cos(theta1);
    J22 =  l2 * cos(theta2);
    J = [J11, J12; J21, J22];
         
    % --- 3. VIRTUAL MODEL CONTROL (EQUAZIONE 19 COMPLETA) ---
    % Vettori Desiderati (d)
    p_d = [-delta_s; -h_cmd]; % Posizione desiderata ruota rispetto all'anca
    v_d = [0; 0];             % Velocità relativa desiderata
    
    % Vettori Effettivi (f) - (In simulazione ideale coincidono con i desiderati)
    p_f = [-xk; -yk];         
    v_f = [0; 0];             
    
    % Calcolo Feedback (Molla e Smorzatore)
    F_feedback = k_p * (p_d - p_f) + k_d * (v_d - v_f);
    
    % Calcolo Feedforward (Forze ottimali dall'MPC)
    F_feedforward = [0; Fz_cmd]; 
    tau_ff = J' * F_feedforward;
    
    % EQUAZIONE 19
    Tau_abs = J' * F_feedback + tau_ff; 
    
    % --- 4. Coppie Relative dei Giunti ---
    tau_hip_log(k) = Tau_abs(2); 
    tau_knee_log(k) = Tau_abs(1) - Tau_abs(2); 
end

%% 6. Plot dei Risultati (MPC)
figure('Name', 'MPC Upper-Body Control', 'Position', [50, 50, 1000, 800]);

% 1. Spostamento (S)
subplot(3, 2, 1); 
plot(t_vec, x_log(1, :), 'b-', 'LineWidth', 2); hold on; 
plot(t_vec, x_ref(1, :), 'r--', 'LineWidth', 1.5);
title('Spostamento Orizzontale (s)'); xlabel('Tempo [s]'); ylabel('[m]'); 
legend('Reale', 'Riferimento', 'Location', 'best'); grid on;

% 2. Altezza (Z)
subplot(3, 2, 2); 
plot(t_vec, x_log(3, :), 'b-', 'LineWidth', 2); hold on; 
plot(t_vec, x_ref(3, :), 'r--', 'LineWidth', 1.5);
title('Altezza CoM (z)'); xlabel('Tempo [s]'); ylabel('[m]'); grid on;

% 3. Errore di Tracking (RIPRISTINATO)
subplot(3, 2, 3); 
plot(t_vec, e_log(1, :), 'g-', 'LineWidth', 1.5); hold on; 
plot(t_vec, e_log(3, :), 'm-', 'LineWidth', 1.5);
title('Errore di Tracking'); xlabel('Tempo [s]'); ylabel('Errore [m]'); 
legend('Errore s', 'Errore z', 'Location', 'best'); grid on;

% 4. Controllo Orizzontale (Delta s)
subplot(3, 2, 4); 
plot(t_vec(1:end-1), u_log(1, 1:end-1), 'k-', 'LineWidth', 1.5);
title('Controllo Orizzontale (\Delta s)'); xlabel('Tempo [s]'); ylabel('[m]'); grid on;

% 5. Forza Inerziale Verticale (F_z)
subplot(3, 2, 5); 
plot(t_vec(1:end-1), u_log(2, 1:end-1), 'k-', 'LineWidth', 1.5);
title('Forza Inerziale Verticale (F_z)'); xlabel('Tempo [s]'); ylabel('[N]'); grid on;

% 6. Andamento Cost Function (J)
subplot(3, 2, 6); 
plot(t_vec(1:end-1), J_log(1:end-1), 'c-', 'LineWidth', 1.5);
title('Andamento Cost Function (J)'); xlabel('Tempo [s]'); ylabel('Costo'); grid on;

sgtitle('Risultati Simulazione: Tracking dell''Upper-Body tramite MPC Adattivo');
%% 7. Plot dei Giunti (Task-Space Controller)
figure('Name', 'Stance Controller - Sforzo Motori Leg', 'Position', [1050, 50, 600, 600]);

subplot(2, 1, 1); plot(t_vec(1:end-1), tau_hip_log, 'b-', 'LineWidth', 2);
title('Coppia Motore Anca (\tau_{hip})'); xlabel('Tempo [s]'); ylabel('Coppia [Nm]'); grid on;

subplot(2, 1, 2); plot(t_vec(1:end-1), tau_knee_log, 'r-', 'LineWidth', 2);
title('Coppia Motore Ginocchio (\tau_{knee})'); xlabel('Tempo [s]'); ylabel('Coppia [Nm]'); grid on;

%% 8. Animazione del Sistema e Video
disp('Generazione dell''animazione (gamba snodata) e salvataggio video...');
video_filename = 'WheeledBiped_MPC_Full_Animation.mp4';
v = VideoWriter(video_filename, 'MPEG-4'); v.FrameRate = 25; open(v);
figure('Name', 'Animazione Robot Completa', 'Position', [200, 200, 800, 500]);
r_wheel = 0.127; skip = round((1/dt) / v.FrameRate); 

for k = 1:skip:steps-1
    clf; hold on;
    s_com = x_log(1, k); z_com = x_log(3, k);
    delta_s = u_log(1, k);
    s_wheel = s_com - delta_s; z_wheel = r_wheel;
    
    % Cinematica Ginocchio per il Disegno
    x_knee = s_wheel + l1 * cos(theta1_log(k));
    z_knee = z_wheel + l1 * sin(theta1_log(k));
    
    plot([-1, 3], [0, 0], 'k-', 'LineWidth', 2); % Suolo
    
    % Disegno ruota
    theta_w = linspace(0, 2*pi, 50);
    plot(s_wheel + r_wheel*cos(theta_w), z_wheel + r_wheel*sin(theta_w), 'k-', 'LineWidth', 2);
    plot(s_wheel, z_wheel, 'k.', 'MarkerSize', 15); 
    
    % Disegno Gamba Piegata (Tibia e Coscia)
    plot([s_wheel, x_knee], [z_wheel, z_knee], 'r-', 'LineWidth', 4); % Tibia
    plot([x_knee, s_com], [z_knee, z_com], 'r-', 'LineWidth', 4);     % Coscia
    plot(x_knee, z_knee, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'y'); % Snodo Ginocchio
    
    % Disegno Busto (CoM) e Target
    plot(s_com, z_com, 'bs', 'MarkerSize', 24, 'MarkerFaceColor', 'b');
    plot(x_ref(1, k), x_ref(3, k), 'gx', 'MarkerSize', 12, 'LineWidth', 2);
    
    axis equal; xlim([-0.5, 2.0]); ylim([-0.1, 1.2]);
    title(sprintf('Animazione Virtual Model Control - Tempo: %.2f s', t_vec(k)));
    xlabel('Spostamento [m]'); ylabel('Altezza [m]'); grid on;
    
    drawnow; writeVideo(v, getframe(gcf)); 
end
close(v); disp(['Animazione terminata. Video salvato come: ', video_filename]);

%% FUNZIONE MINIMUM JERK
function [pos, vel] = genera_profilo_quinto_ordine(t, target, T_totale)
    if t <= 0
        pos = 0; vel = 0;
    elseif t >= T_totale
        pos = target; vel = 0;
    else
        tau = t / T_totale;
        pos = target * (10*tau^3 - 15*tau^4 + 6*tau^5);
        vel = (target / T_totale) * (30*tau^2 - 60*tau^3 + 30*tau^4);
    end
end