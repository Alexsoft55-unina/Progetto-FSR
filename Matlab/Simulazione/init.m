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



%% Generazione traiettoria 
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
    x_ref(4,k) = 0;
end
ref_ts = timeseries(x_ref, t_vec);
x_ref_global = x_ref;

%% MPC 
%% 2. Pesi della Funzione Obiettivo e Matrici fisse
S = diag([1, 1, 1000, 1]); % Pesi alti sulle posizioni
W = diag([1, 0.00001]);           % Pesi sugli ingressi

S_bar = 10*kron(eye(N), S);
W_bar = kron(eye(N), W);

%% Task Space Controller
l1 = 0.45; % Lunghezza Tibia [m]
l2 = 0.45; % Lunghezza Coscia [m]



% Parametri Virtual Model Control (Eq. 19)
k_p = diag([2000, 2000]); % Matrice di Stiffness virtuale (X, Z) [N/m]
k_d = diag([100, 100]);   % Matrice di Damping virtuale (X, Z) [Ns/m]
