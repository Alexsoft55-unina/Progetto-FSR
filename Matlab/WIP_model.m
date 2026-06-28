%% Simulazione Completa VL-WIP con Traiettoria a Minimum Jerk (5° Ordine)
% Basato su: "Modeling and Control of a Wheeled Biped Robot" (MDPI)

clear; close all; clc;

%% 1. Parametri Fisici Costanti e Linearizzazione
% --- Parametri Fisici Costanti ---
m_w = 3.5;      % Mass of the wheel [kg]
I_w = 0.1;      % Moment of inertia of the wheel [kg*m^2]
r = 0.127;      % Radius of the wheel [m]
d = 0.63;       % Distance between two wheels [m]
m_b = 73;       % Mass of the upper body [kg]
I_z = 3.3;      % Moment of inertia about the z-axis [kg*m^2]
g = 9.81;       % Accelerazione di gravità [m/s^2]

% Variabile simbolica per la lunghezza (permetterà il calcolo del TV-LQR in futuro)
syms l real
I_y = (1/3) * m_b * l^2;  % Moment of inertia about the y-axis

% --- Calcolo dei Coefficienti Matrici (simbolici in funzione di l) ---
den_comune = 2*I_w*(I_y + m_b*l^2) + (2*l^2*m_b*m_w + I_y*(m_b + 2*m_w)*r^2);

a_1 = - (g * l^2 * m_b^2 * r^2) / den_comune;
a_2 = (g * l * m_b * (2*I_w + (m_b + 2*m_w)*r^2)) / den_comune;
b_1 = (r * (I_y + l*m_b*(l + r))) / den_comune;
b_2 = - (2*I_w + r*(l*m_b + (m_b + 2*m_w)*r)) / den_comune;
b_3 = (d * r) / (2*I_z*r^2 + d^2*(I_w + m_w*r^2));

% Matrici di stato e ingresso simboliche
A = [0,   0,   0,   1,   0,   0;
     0,   0,   0,   0,   1,   0;
     0,   0,   0,   0,   0,   1;
     0, a_1,   0,   0,   0,   0;
     0, a_2,   0,   0,   0,   0;
     0,   0,   0,   0,   0,   0];

B = [  0,    0;
       0,    0;
       0,    0;
     b_1,  b_1;
     b_2,  b_2;
     b_3, -b_3];

% Punto Operativo Nominale (altezza fissa per ora)
l_nom = 0.5; % [m] 

% Struttura parametri per la simulazione non lineare
params.m_b = m_b; params.m_w = m_w; params.I_w = I_w;
params.l = l_nom; params.r = r;     params.d = d; 
params.I_z = I_z; params.I_y = double(subs(I_y, l, l_nom));

% Valutazione numerica delle matrici A e B
A_num = double(subs(A, l, l_nom));
B_num = double(subs(B, l, l_nom));

%% 2. Analisi di Controllabilità e Sintesi LQR
Co = ctrb(A_num, B_num);
if rank(Co) < size(A_num, 1)
    error('Il sistema linearizzato non è completamente controllabile.');
end

% Pesi sugli stati: X = [s; theta; phi; s_dot; theta_dot; phi_dot]
Q = diag([10, 1000, 10, 1, 100, 1]); 
R = diag([0.1, 0.1]); 

% Matrice dei guadagni K
[K, ~, ~] = lqr(A_num, B_num, Q, R);
disp('Matrice dei guadagni K calcolata con successo.');
Bv = K; % Precompensazione per il tracking

%% 3. Setup Traiettoria e Simulazione Non Lineare
X0 = [0; 0; 0; 0; 0; 0]; % Condizioni iniziali

% Target di movimento finali
s_target = 1.0;           % Spostamento desiderato [m]
phi_target = deg2rad(10); % Imbardata desiderata [rad]

% Limiti fisici (dal paper)
v_max_lim = 0.8;          % m/s
omega_max_lim = 0.5;      % rad/s

% Il tempo totale 
T_totale = 5; 
disp(['Tempo totale della manovra impostato: ', num2str(T_totale), ' secondi']);

% Allunghiamo la simulazione di 5 secondi oltre la fine del movimento per vedere l'assestamento
t_span = [0, T_totale + 5]; 

% Wrapper aggiornato
dinamica_chiusa = @(t, X) dinamica_non_lineare_wrapper(t, X, s_target, phi_target, T_totale, K, Bv, params);

options = odeset('RelTol', 1e-4, 'AbsTol', 1e-6);
[t_sim, X_sim] = ode45(dinamica_chiusa, t_span, X0, options);

%% 4. Analisi dei Risultati (Stati e Sforzo di Controllo)
% -- Plot degli Stati --
figure('Name', 'Stati del Robot', 'Color', 'w');
subplot(3,1,1);
plot(t_sim, X_sim(:,1), 'b', 'LineWidth', 1.5);
ylabel('Posizione s [m]'); title('Spostamento Lineare'); grid on;

subplot(3,1,2);
plot(t_sim, rad2deg(X_sim(:,2)), 'r', 'LineWidth', 1.5);
ylabel('Angolo \theta [deg]'); title('Angolo di Tilt (Pendolo)'); grid on;

subplot(3,1,3);
plot(t_sim, rad2deg(X_sim(:,3)), 'k', 'LineWidth', 1.5);
ylabel('Angolo \phi [deg]'); xlabel('Tempo [s]'); title('Angolo di Imbardata (Yaw)'); grid on;

% -- Calcolo e Plot Sforzo di Controllo (Motori) --
tau_history = zeros(length(t_sim), 2);
for i = 1:length(t_sim)
    t_curr = t_sim(i);
    X_curr = X_sim(i, :)';
    
    % Ricostruiamo X_des istantaneo usando il polinomio di 5° ordine
    [s_des, s_dot_des] = genera_profilo_quinto_ordine(t_curr, s_target, T_totale);
    [phi_des, phi_dot_des] = genera_profilo_quinto_ordine(t_curr, phi_target, T_totale);
    
    X_des_curr = [s_des; 0; phi_des; s_dot_des; 0; phi_dot_des];
    
    % Ricalcolo della coppia esatta applicata in quel momento
    tau_history(i, :) = (-K * X_curr + Bv * X_des_curr)';
end

figure('Name', 'Sforzo di Controllo', 'Color', 'w');
subplot(2,1,1);
plot(t_sim, tau_history(:,1), 'b', 'LineWidth', 2);
ylabel('\tau_l [Nm]'); title('Coppia Motore - Ruota Sinistra (Minimum Jerk)'); grid on;

subplot(2,1,2);
plot(t_sim, tau_history(:,2), 'r', 'LineWidth', 2);
ylabel('\tau_r [Nm]'); xlabel('Tempo [s]'); title('Coppia Motore - Ruota Destra (Minimum Jerk)'); grid on;

%% 5. Animazione 3D e Salvataggio Video
disp('Generazione del video in corso...');
video_filename = 'WIP_Simulation_Completa.mp4';
v = VideoWriter(video_filename, 'MPEG-4');
v.FrameRate = 30; open(v);

fig_anim = figure('Name', 'Animazione VL-WIP', 'Color', 'w', 'Position', [100, 100, 800, 600]);
hold on; grid on; axis equal; view(3);
xlim([-0.5 1.5]); ylim([-0.5 1.5]); zlim([0 1.2]);
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]'); title('Simulazione Dinamica VL-WIP');

l_anim = params.l;
h_axle = plot3([0 0], [0 0], [0 0], 'k-', 'LineWidth', 6);
h_rod  = plot3([0 0], [0 0], [0 0], 'b-', 'LineWidth', 4);
h_mass = plot3(0, 0, 0, 'ko', 'MarkerSize', 15, 'MarkerFaceColor', 'r');
h_wheelL = plot3(0, 0, 0, 'k-', 'LineWidth', 3);
h_wheelR = plot3(0, 0, 0, 'k-', 'LineWidth', 3);
h_path = plot3(0, 0, 0, 'g--', 'LineWidth', 1.5);

t_video = 0:(1/v.FrameRate):t_sim(end);
X_video = interp1(t_sim, X_sim, t_video);
path_x = zeros(1, length(t_video)); path_y = zeros(1, length(t_video));

for i = 1:length(t_video)
    s = X_video(i, 1); theta = X_video(i, 2); phi = X_video(i, 3);
    
    x_c = s * cos(phi); y_c = s * sin(phi); z_c = r;
    path_x(i) = x_c; path_y(i) = y_c;
    
    dx_trans = (d/2) * -sin(phi); dy_trans = (d/2) * cos(phi);
    x_L = x_c + dx_trans; y_L = y_c + dy_trans;
    x_R = x_c - dx_trans; y_R = y_c - dy_trans;
    
    x_p = x_c + l_anim * sin(theta) * cos(phi);
    y_p = y_c + l_anim * sin(theta) * sin(phi);
    z_p = r + l_anim * cos(theta);
    
    set(h_axle, 'XData', [x_L x_R], 'YData', [y_L y_R], 'ZData', [r r]);
    set(h_rod, 'XData', [x_c x_p], 'YData', [y_c y_p], 'ZData', [z_c z_p]);
    set(h_mass, 'XData', x_p, 'YData', y_p, 'ZData', z_p);
    set(h_path, 'XData', path_x(1:i), 'YData', path_y(1:i), 'ZData', repmat(r, 1, i));
    
    angoli = linspace(0, 2*pi, 20);
    wxL = x_L + r * cos(angoli) * cos(phi); wyL = y_L + r * cos(angoli) * sin(phi); wzL = r + r * sin(angoli);
    set(h_wheelL, 'XData', wxL, 'YData', wyL, 'ZData', wzL);
    wxR = x_R + r * cos(angoli) * cos(phi); wyR = y_R + r * cos(angoli) * sin(phi); wzR = r + r * sin(angoli);
    set(h_wheelR, 'XData', wxR, 'YData', wyR, 'ZData', wzR);
    
    drawnow; writeVideo(v, getframe(fig_anim));
end
close(v);
disp(['Video salvato con successo: ', fullfile(pwd, video_filename)]);

%% --- FUNZIONI DI SUPPORTO ---

function dX = dinamica_non_lineare_wrapper(t, X, s_tgt, phi_tgt, T_totale, K, Bv, params)
    % 1. Genera Riferimenti Istante per Istante (Minimum Jerk)
    [s_des, s_dot_des] = genera_profilo_quinto_ordine(t, s_tgt, T_totale);
    [phi_des, phi_dot_des] = genera_profilo_quinto_ordine(t, phi_tgt, T_totale);
    
    X_des = [s_des; 0; phi_des; s_dot_des; 0; phi_dot_des];
    
    % 2. Azione di controllo (LQR)
    tau_w = -K * X + Bv * X_des; 
    
    % 3. Dinamica Non Lineare
    dX = dinamica_non_lineare(t, X, tau_w, params);
end

function dX = dinamica_non_lineare(~, X, tau_w, params)
    theta = X(2); theta_dot = X(5);
    mb = params.m_b; mw = params.m_w; Iw = params.I_w;
    l = params.l; r = params.r; d = params.d; 
    Iy = params.I_y; Iz = params.I_z; g = 9.81;

    M11 = mb + 2*mw + 2*(Iw/r^2);
    M12 = mb*l*cos(theta);
    M22 = mb*l^2 + Iy;
    M33 = (d^2*mw)/2 + (d^2*Iw)/(2*r^2) + Iz;
    
    M = [M11, M12, 0; M12, M22, 0; 0, 0, M33];
    C = [-mb * l * (theta_dot^2) * sin(theta); -mb * g * l * sin(theta); 0];
    B_nl = [1/r, 1/r; -1, -1; d/(2*r), -d/(2*r)];

    phi_ddot = M \ (B_nl * tau_w - C);
    dX = [X(4:6); phi_ddot];
end

function [pos, vel] = genera_profilo_quinto_ordine(t, target, T_totale)
    if target == 0
        pos = 0; vel = 0; return;
    end
    
    if t <= 0
        pos = 0; vel = 0;
    elseif t >= T_totale
        pos = target; vel = 0;
    else
        tau = t / T_totale;
        % Polinomio 5° grado per posizione
        pos = target * (10*tau^3 - 15*tau^4 + 6*tau^5);
        % Derivata per velocità
        vel = (target / T_totale) * (30*tau^2 - 60*tau^3 + 30*tau^4);
    end
end