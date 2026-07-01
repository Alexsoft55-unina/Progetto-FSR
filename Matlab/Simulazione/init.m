clear; clc; close all;

%% 1. Parametri del Sistema e Assunzioni


load Equazione_Dinamica_Robot_2D.mat

% --- Aggiungi questa dichiarazione ---
syms qw q1 q2 q3 dq_w dq1 dq2 dq3 real
% Creazione degli array per la sostituzione
% Assicurati che le variabili simboliche (old_vars) siano state dichiarate tramite 'syms'
syms m1 m2 m3 mw l1 l2 l3 Rw g Iw I1 I2 I3 real

old_vars = [m1, m2, m3, mw, l1, l2, l3, Rw, g, Iw, I1, I2, I3];
new_vals = [10, 10, 60,  5, 0.5, 0.5, 0.5, 0.15, 9.81, 0.05, 0.1, 0.1, 0.1]; % Aggiorna questi valori!

% Applicazione dei parametri alle matrici
M_param = subs(M_matrix, old_vars, new_vals);
C_param = subs(C_matrix, old_vars, new_vals);
G_param = subs(G_matrix, old_vars, new_vals);

% Semplificazione algebrica opzionale ma consigliata
M_param = simplify(M_param);
C_param = simplify(C_param);
G_param = simplify(G_param);

mb = 73;                % Massa dell'upper body [kg] 
m3 = 60;
g = 9.81;               % Accelerazione di gravità [m/s^2]
dt = 0.01;              % Tempo di campionamento [s] (100 Hz)
N = 15;                 % Orizzonte di predizione

% Limiti fisici
mu = 0.6;               % Coefficiente di attrito 
L_max = 0.8;            % Lunghezza massima della gamba [m]
zb = 0.6;               % Distanza verticale asse-anca [m]
F_min = 100;            % Forza z minima [N]
F_max = 1500;           % Forza z massima [N]


% Assicurati che i vettori q e dq contengano le stesse variabili simboliche 
% presenti nelle tue matrici M_param, C_param, G_param.
% Esempio (adatta i nomi se le tue variabili simboliche si chiamano diversamente):
q_vec = [qw; q1; q2; q3]; 
dq_vec = [dq_w; dq1; dq2; dq3]; 

% Generazione del file funzione ottimizzato
disp('Generazione della funzione numerica per Simulink in corso...');
matlabFunction(M_param, C_param, G_param, ...
    'File', 'Dinamica_Robot', ...
    'Vars', {q_vec, dq_vec}, ...      % Ingressi della funzione: vettori q e dq
    'Outputs', {'M', 'C', 'G'}, ...   % Uscite della funzione
    'Optimize', true);                % Ottimizza il codice per la simulazione
disp('File Dinamica_Robot.m generato con successo!');


% Stato: x = [s; s_dot; z; z_dot]
% Ingresso: u = [Delta_s; F_z]
dc = [0; 0; 0; -g];     % Disturbo di gravità
dk = dc * dt;

robot_params.mb = 73.0;    % Massa del corpo superiore (kg)
robot_params.Iz = 3.3;     % Momento di inerzia rispetto all'asse z (kg*m^2)

robot_params.m1 = 1.2;     % Massa dello stinco (kg)
robot_params.m2 = 5.3;     % Massa della coscia (kg)
robot_params.m3 = 60.0;    % Massa del torso (kg)

robot_params.l1 = 0.45;    % Lunghezza dello stinco (m)
robot_params.l2 = 0.45;    % Lunghezza della coscia (m)
robot_params.l3 = 0.35;    % Altezza del torso (m)
% Nota per Iy: la tabella indica (1/3)*mb*l^2. Assumo che 'l' sia l3 (torso).
% Se è un'altra lunghezza, dovrai aggiornare questa formula.
robot_params.Iy = (1/3) * robot_params.mb * (robot_params.l3)^2;
% Calcolo dinamico di I1 e I2 (Asta sottile)
robot_params.I1 = (1/12) * robot_params.m1 * (robot_params.l1)^2;
robot_params.I2 = (1/12) * robot_params.m2 * (robot_params.l2)^2;

% Assicurati di aggiungere anche lc1 e lc2 (distanza baricentro)
% Per un'asta omogenea, il baricentro è a metà lunghezza
robot_params.lc1 = robot_params.l1 / 2;
robot_params.lc2 = robot_params.l2 / 2;

% CINEMATICA_INVERSA_2R Calcola gli angoli dei giunti per raggiungere un tar
q_init = cinematica_inversa_2R(0, 0.6, robot_params);

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
