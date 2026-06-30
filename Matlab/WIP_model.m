%% Simulazione Completa VL-WIP con TV-LQR e Traiettoria a Minimum Jerk
% Basato su: "Modeling and Control of a Wheeled Biped Robot" (MDPI)

clear; close all; clc;

%% 1. Parametri Fisici Costanti e Linearizzazione Simbolica
% --- Parametri Fisici Costanti ---
m_w = 3.5;      % Mass of the wheel [kg]
I_w = 0.1;      % Moment of inertia of the wheel [kg*m^2]
r = 0.127;      % Radius of the wheel [m]
d = 0.63;       % Distance between two wheels [m]
m_b = 73;       % Mass of the upper body [kg]
I_z = 3.3;      % Moment of inertia about the z-axis [kg*m^2]
g = 9.81;       % Accelerazione di gravità [m/s^2]

% Variabile simbolica per la lunghezza
syms l_sym real
I_y_sym = (1/3) * m_b * l_sym^2;  % Moment of inertia about the y-axis

% --- Calcolo dei Coefficienti Matrici (simbolici in funzione di l) ---
den_comune = 2*I_w*(I_y_sym + m_b*l_sym^2) + (2*l_sym^2*m_b*m_w + I_y_sym*(m_b + 2*m_w)*r^2);

a_1 = - (g * l_sym^2 * m_b^2 * r^2) / den_comune;
a_2 = (g * l_sym * m_b * (2*I_w + (m_b + 2*m_w)*r^2)) / den_comune;
b_1 = (r * (I_y_sym + l_sym*m_b*(l_sym + r))) / den_comune;
b_2 = - (2*I_w + r*(l_sym*m_b + (m_b + 2*m_w)*r)) / den_comune;
b_3 = (d * r) / (2*I_z*r^2 + d^2*(I_w + m_w*r^2));

% Matrici di stato e ingresso simboliche
A_sym = [0,   0,   0,   1,   0,   0;
         0,   0,   0,   0,   1,   0;
         0,   0,   0,   0,   0,   1;
         0, a_1,   0,   0,   0,   0;
         0, a_2,   0,   0,   0,   0;
         0,   0,   0,   0,   0,   0];

B_sym = [  0,    0;
           0,    0;
           0,    0;
         b_1,  b_1;
         b_2,  b_2;
         b_3, -b_3];

%% 2. Generazione della Look-Up Table (LUT) per il TV-LQR
disp('Generazione della Look-Up Table (LUT) in corso...');

% Definiamo un range operativo per la lunghezza del pendolo
l_range = linspace(0.3, 0.8, 50); % 50 punti tra 0.3m e 0.8m
K_LUT = zeros(length(l_range), 2, 6); % Matrice 3D per salvare i guadagni K

% Pesi sugli stati: X = [s; theta; phi; s_dot; theta_dot; phi_dot]
Q = diag([10, 1000, 10, 1, 100, 1]); 
R = diag([0.1, 0.1]); 

for i = 1:length(l_range)
    l_val = l_range(i);
    
    % Valutazione numerica delle matrici A e B per il valore corrente di l
    A_num = double(subs(A_sym, l_sym, l_val));
    B_num = double(subs(B_sym, l_sym, l_val));
    
    % Calcolo del guadagno K e salvataggio nella LUT
    [K_val, ~, ~] = lqr(A_num, B_num, Q, R);
    K_LUT(i, :, :) = K_val;
end
disp('LUT generata con successo.');

%% 3. Setup Traiettoria e Simulazione Non Lineare
X0 = [0; 0; 0; 0; 0; 0]; % Condizioni iniziali

% Target di movimento finali
s_target = 1.0;           % Spostamento desiderato [m]
phi_target = deg2rad(10); % Imbardata desiderata [rad]

T_totale = 5.0; % [secondi] Tempo di manovra rilassato
t_span = [0, 10]; 

% Struttura parametri fisici base (l e I_y verranno aggiornati dinamicamente)
params_base.m_b = m_b; params_base.m_w = m_w; params_base.I_w = I_w;
params_base.r = r;     params_base.d = d;     params_base.I_z = I_z;

% Funzione per la lunghezza variabile nel tempo (es. movimento sinusoidale)
% l(t) = l_nom + ampiezza * sin(omega * t)
l_nom = 0.5;
ampiezza_l = 0.15;
omega_l = 2 * pi / 3; % Frequenza del movimento (es. 3 secondi per ciclo)
funzione_l = @(t) l_nom + ampiezza_l * sin(omega_l * t);

% Wrapper aggiornato per includere la LUT e la lunghezza variabile
dinamica_chiusa = @(t, X) dinamica_non_lineare_wrapper_tv(t, X, s_target, phi_target, T_totale, l_range, K_LUT, funzione_l, params_base);

options = odeset('RelTol', 1e-4, 'AbsTol', 1e-6);
[t_sim, X_sim] = ode45(dinamica_chiusa, t_span, X0, options);

%% 4. Analisi dei Risultati (Stati e Sforzo di Controllo)
figure('Name', 'Stati del Robot', 'Color', 'w');
subplot(4,1,1);
plot(t_sim, X_sim(:,1), 'b', 'LineWidth', 1.5);
ylabel('Posizione s [m]'); title('Spostamento Lineare'); grid on;

subplot(4,1,2);
plot(t_sim, rad2deg(X_sim(:,2)), 'r', 'LineWidth', 1.5);
ylabel('Angolo \theta [deg]'); title('Angolo di Tilt'); grid on;

subplot(4,1,3);
plot(t_sim, rad2deg(X_sim(:,3)), 'k', 'LineWidth', 1.5);
ylabel('Angolo \phi [deg]'); title('Angolo di Imbardata'); grid on;

% Calcoliamo l(t) per tracciarlo
l_history = arrayfun(funzione_l, t_sim);
subplot(4,1,4);
plot(t_sim, l_history, 'g', 'LineWidth', 1.5);
ylabel('Altezza l [m]'); xlabel('Tempo [s]'); title('Variazione Lunghezza Pendolo'); grid on;

% -- Calcolo e Plot Sforzo di Controllo (Motori) --
tau_history = zeros(length(t_sim), 2);
for i = 1:length(t_sim)
    t_curr = t_sim(i);
    X_curr = X_sim(i, :)';
    
    % Ricalcolo lunghezza e interpolazione K
    l_curr = funzione_l(t_curr);
K_curr = squeeze(interp1(l_range, K_LUT, l_curr, 'linear', 'extrap'));
Bv_curr = K_curr; 
    
    [s_des, s_dot_des] = genera_profilo_quinto_ordine(t_curr, s_target, T_totale);
    [phi_des, phi_dot_des] = genera_profilo_quinto_ordine(t_curr, phi_target, T_totale);
    X_des_curr = [s_des; 0; phi_des; s_dot_des; 0; phi_dot_des];
    
    tau_history(i, :) = (-K_curr * X_curr + Bv_curr * X_des_curr)';
end

figure('Name', 'Sforzo di Controllo', 'Color', 'w');
subplot(2,1,1);
plot(t_sim, tau_history(:,1), 'b', 'LineWidth', 2);
ylabel('\tau_l [Nm]'); title('Coppia Motore - Ruota Sinistra (TV-LQR)'); grid on;

subplot(2,1,2);
plot(t_sim, tau_history(:,2), 'r', 'LineWidth', 2);
ylabel('\tau_r [Nm]'); xlabel('Tempo [s]'); title('Coppia Motore - Ruota Destra (TV-LQR)'); grid on;

%% 5. Animazione 3D e Salvataggio Video
% ... (Mantieni il tuo codice di animazione qui, assicurandoti di usare l_history(i) per la z_p invece di l_anim fisso) ...
disp('Generazione del video in corso...');
video_filename = 'WIP_Simulation_Completa.mp4';
v = VideoWriter(video_filename, 'MPEG-4');
v.FrameRate = 30; open(v);

fig_anim = figure('Name', 'Animazione VL-WIP', 'Color', 'w', 'Position', [100, 100, 800, 600]);
hold on; grid on; axis equal; view(3);
xlim([-0.5 1.5]); ylim([-0.5 1.5]); zlim([0 1.2]);
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]'); title('Simulazione Dinamica VL-WIP');

h_axle = plot3([0 0], [0 0], [0 0], 'k-', 'LineWidth', 6);
h_rod  = plot3([0 0], [0 0], [0 0], 'b-', 'LineWidth', 4);
h_mass = plot3(0, 0, 0, 'ko', 'MarkerSize', 15, 'MarkerFaceColor', 'r');
h_wheelL = plot3(0, 0, 0, 'k-', 'LineWidth', 3);
h_wheelR = plot3(0, 0, 0, 'k-', 'LineWidth', 3);
h_path = plot3(0, 0, 0, 'g--', 'LineWidth', 1.5);

t_video = 0:(1/v.FrameRate):t_sim(end);
X_video = interp1(t_sim, X_sim, t_video);
l_video = arrayfun(funzione_l, t_video); % Interpoliamo anche l(t) per l'animazione
path_x = zeros(1, length(t_video)); path_y = zeros(1, length(t_video));

for i = 1:length(t_video)
    s = X_video(i, 1); theta = X_video(i, 2); phi = X_video(i, 3);
    l_anim = l_video(i); % Lunghezza al frame corrente
    
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

function dX = dinamica_non_lineare_wrapper_tv(t, X, s_tgt, phi_tgt, T_totale, l_range, K_LUT, funzione_l, params)
    
    % 1. Determiniamo la lunghezza attuale l(t)
    l_curr = funzione_l(t);
    
    % Aggiorniamo i parametri dipendenti da l
    params.l = l_curr;
    params.I_y = (1/3) * params.m_b * l_curr^2;
    
    % 2. Interpoliamo K dalla LUT
    % Usiamo interpn per interpolare lungo la prima dimensione di K_LUT
    % K_LUT ha dimensioni (n_punti_l, 2, 6). 'squeeze' rimuove la dimensione di l.
K_curr = squeeze(interp1(l_range, K_LUT, l_curr, 'linear', 'extrap'));
Bv_curr = K_curr;
    
    % 3. Genera Riferimenti Istante per Istante (Minimum Jerk)
    [s_des, s_dot_des] = genera_profilo_quinto_ordine(t, s_tgt, T_totale);
    [phi_des, phi_dot_des] = genera_profilo_quinto_ordine(t, phi_tgt, T_totale);
    
    X_des = [s_des; 0; phi_des; s_dot_des; 0; phi_dot_des];
    
    % 4. Azione di controllo (TV-LQR)
    tau_w = -K_curr * X + Bv_curr * X_des; 
    
    % 5. Dinamica Non Lineare
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
        pos = target * (10*tau^3 - 15*tau^4 + 6*tau^5);
        vel = (target / T_totale) * (30*tau^2 - 60*tau^3 + 30*tau^4);
    end
end