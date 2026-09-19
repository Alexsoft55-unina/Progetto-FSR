%% 1. Parametri fisici
% --- Parametri Base / Ruote ---
robot_params.mw = 3.5;      % massa ruota
robot_params.Iw = 0.1;      % inerzia ruota
robot_params.d  = 0.63;     % distanza / asse
robot_params.Rw = 0.127;    % raggio ruota
robot_params.Iz = 3.3;      % inerzia asse Z globale (presunta)
robot_params.g  = 9.81;     % accelerazione di gravità

% --- Parametri di Massa e Lunghezza ---
robot_params.m1 = 1.2;      % massa stinco (shank)
robot_params.m2 = 5.3;      % massa coscia (thigh)
robot_params.m3 = 60.0/10;     % massa torso

robot_params.l1 = 0.45;     % lunghezza stinco
robot_params.l2 = 0.45;     % lunghezza coscia
robot_params.l3 = 0.35;     % parametro base per dimensioni torso

% =========================================================================
% MODELLO 3D: Inerzie complete come vettori diagonali [Ix, Iy, Iz]
% =========================================================================

% Stinco (Modello: Cilindro solido lungo Z)
robot_params.radius_stinco = 1e-2;
robot_params.inertia_stinco = [ ...
    (1/12) * robot_params.m1 * (3 * robot_params.radius_stinco^2 + robot_params.l1^2), ...
    (1/12) * robot_params.m1 * (3 * robot_params.radius_stinco^2 + robot_params.l1^2), ...
    (1/2) * robot_params.m1 * robot_params.radius_stinco^2 ];

% Coscia (Modello: Cilindro solido lungo Z)
robot_params.radius_coscia = 1e-2;
robot_params.inertia_coscia = [ ...
    (1/12) * robot_params.m2 * (3 * robot_params.radius_coscia^2 + robot_params.l2^2), ...
    (1/12) * robot_params.m2 * (3 * robot_params.radius_coscia^2 + robot_params.l2^2), ...
    (1/2) * robot_params.m2 * robot_params.radius_coscia^2 ];

% Torso (Modello: Parallelepipedo)
% Dimensioni: x = 0.35, y = 0.70, z = 0.35
robot_params.length_torso = [robot_params.l3, 2 * robot_params.l3, robot_params.l3];
robot_params.inertia_torso = (1/12) * robot_params.m3 * [ ...
    robot_params.length_torso(2)^2 + robot_params.length_torso(3)^2, ...
    robot_params.length_torso(1)^2 + robot_params.length_torso(3)^2, ...
    robot_params.length_torso(1)^2 + robot_params.length_torso(2)^2 ];

% =========================================================================
% MODELLO 1D (Semplificato): Momenti d'inerzia planari
% NOTA: Da utilizzare in alternativa al modello 3D (Asta sottile lungo Z)
% =========================================================================
robot_params.I1 = (1/12) * robot_params.m1 * robot_params.l1^2;
robot_params.I2 = (1/12) * robot_params.m2 * robot_params.l2^2;
robot_params.I3 = (1/12) * robot_params.m3 * robot_params.l3^2; % Attenzione: sottostima l'inerzia del torso

disp('Parametri fisici caricati e calcolati con successo.');