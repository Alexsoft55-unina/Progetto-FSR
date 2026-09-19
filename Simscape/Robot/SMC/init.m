%% Initial Joint Position

q_init = [pi/6,-pi/3, 0,pi/6,-pi/3,0];

%% Sample time actuator

Ts=1e-3;



%% Motor parameters

Motor_damping = 0.025;
Motor_stiffness = 0.0;

%% Robot parameters

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


q0_guess = [pi/3; -pi/6; pi/3];


K.KP = -0.22;   K.KI = -0.06;   K.TH_MAX = deg2rad(12);            % esterno (NEGATIVI)
K.c1 = 6.0;     K.ka = 3.5;     K.kb = 12.0;   K.eps1 = 0.01;      % super-twisting pitch
K.c2 = 12.0;    K.k2 = 60.0;    K.eta2 = 20.0; K.phi2 = 0.05;      % ginocchio
K.c3 = 12.0;    K.k3 = 60.0;    K.eta3 = 20.0; K.phi3 = 0.05;      % torso
K.q2_ref = -0.30;   K.q3_ref = 0.0;
K.TAU_MAX = [25; 120; 120];
K.mu = 0.8;   K.N_nom = (robot_params.mw+robot_params.m1+robot_params.m2+robot_params.m3)*robot_params.g;   K.Rw = robot_params.Rw;

B  = [-1 0 -1; 1 0 0; 0 1 -1; 0 0 1];