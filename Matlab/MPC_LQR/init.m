% =========================================================================
% Script di Inizializzazione Parametri per il Robot Bipede "Scooter"
% Rif: Modeling and Control of a Wheeled Biped Robot (Cui et al., 2022)
% =========================================================================

clc;clear;

robot_params.mw = 3.5;      % Massa della ruota (kg)
robot_params.Iw = 0.1;      % Inerzia della ruota (kg*m^2)
robot_params.d  = 0.63;     % Distanza tra le due ruote (m)

robot_params.Rw = 0.127; 

robot_params.mb = 66.5;%73.0;    % Massa del corpo superiore (kg)
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
robot_params.I3 = (1/12) * robot_params.m3 * (robot_params.l3)^2;
% Assicurati di aggiungere anche lc1 e lc2 (distanza baricentro)
% Per un'asta omogenea, il baricentro è a metà lunghezza
robot_params.lc1 = robot_params.l1 / 2;
robot_params.lc2 = robot_params.l2 / 2;
robot_params.lc3 = robot_params.l3 / 2;
robot_params.g = 9.81;

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
params_num = [robot_params.Rw, robot_params.mw, robot_params.m1, robot_params.m2, robot_params.m3, robot_params.l1, robot_params.l2, robot_params.l3, robot_params.g, robot_params.Iw, robot_params.I1, robot_params.I2, robot_params.I3];

% Generazione di un UNICO file .m fisico (Ottimizzato per Simulink)
disp('Generazione del file calc_dynamics.m in corso...');
matlabFunction(subs(M_matrix, params_sym, params_num), ...
               subs(C_matrix_times_dQ, params_sym, params_num), ...
               subs(G_matrix, params_sym, params_num), ...
               'File', 'calc_dynamics', ...
               'Vars', {Q, dQ}, ...
               'Outputs', {'M', 'CdQ', 'G'});
disp('File calc_dynamics.m generato con successo!');


%% 1. Parametri delle Ruote
m_w = 3.5;       % Massa della ruota [kg]
I_w = 0.1;       % Momento di inerzia della ruota [kg*m^2]
r   = 0.127;     % Raggio della ruota [m]
d   = 0.63;      % Distanza tra le due ruote (carreggiata) [m]

%% 2. Parametri delle Gambe (Singola Gamba)
m_1 = 1.2;       % Massa del polpaccio (shank) [kg]
m_2 = 5.3;       % Massa della coscia (thigh) [kg]
l_1 = 0.45;      % Lunghezza del polpaccio [m]
l_2 = 0.45;      % Lunghezza della coscia [m]

%% 3. Parametri del Torso e Corpo Superiore
m_3 = 60.0;      % Massa del torso [kg]
l_3 = 0.35;      % Altezza del torso [m]
I_z = 3.3;       % Momento di inerzia attorno all'asse z [kg*m^2]

% Nota: La massa equivalente del corpo superiore (m_b) è 73 kg.
% Questo valore è coerente con la somma delle masse delle gambe e del torso:
% m_b = m_3 + 2*(m_1 + m_2) = 60 + 2*(1.2 + 5.3) = 73 kg.
m_b = 73.0;      

%% 4. Costanti Fisiche
g = 9.81;        % Accelerazione di gravità [m/s^2]

%% NOTA SUL CALCOLO DINAMICO:
% Il pendolo ha una lunghezza variabile 'l'. Di conseguenza, il momento 
% di inerzia attorno all'asse y (I_y) non è una costante fissa, ma dipende 
% dalla lunghezza del pendolo secondo la formula: I_y = (1/3) * m_b * l^2.
% Questo calcolo dovrà essere effettuato in tempo reale all'interno di Simulink.

disp('Parametri del robot Scooter caricati con successo nel Workspace.');
% Modello di stato per l'MPC (Eq. 9)
% Stati: [s, s_dot, z, z_dot, Delta_s]
% Input: [Delta_s_dot, F_z]
m_b = 73; % Massa
h0 = 0.6; % Altezza nominale

A_mpc = [0 1 0 0 0;
         0 0 0 0 g/h0;
         0 0 0 1 0;
         0 0 0 0 0;
         0 0 0 0 0];

B_mpc = [0 0;
         0 0;
         0 0;
         0 1/m_b;
         1 0];
         
C_mpc = eye(5);
D_mpc = zeros(5,2);

% Creazione dell'oggetto plant
plant_mpc = ss(A_mpc, B_mpc, C_mpc, D_mpc);