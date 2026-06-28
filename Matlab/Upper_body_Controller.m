% =========================================================================
% Simulazione MPC: Upper-Body Controller per Wheeled Biped Robot
% =========================================================================
clear; clc; close all;

%% 1. Parametri del Sistema e Assunzioni
mb = 73;                % Massa dell'upper body [kg] (Dal PDF)
g = 9.81;               % Accelerazione di gravità [m/s^2]
h = 0.6;                % Altezza verticale del CoM [m] (Valore tipico dal PDF)
dt = 0.01;              % Tempo di campionamento [s] (100 Hz dal PDF)
N = 15;                 % Orizzonte di predizione (Assunzione)

% Limiti (Assunzioni basate sulle restrizioni geometriche e fisiche)
mu = 0.6;               % Coefficiente di attrito 
L_max = 0.8;            % Lunghezza massima della gamba [m]
zb = 0.6;               % Distanza verticale asse-anca [m]
F_min = 100;            % Forza z minima [N]
F_max = 1500;           % Forza z massima [N]

%% 2. Modello Dinamico Spazio di Stato (Continuo e Discreto)
% Stato: x = [s; s_dot; z; z_dot]
% Ingresso: u = [Delta_s; F_z]
% Disturbo affine: dc = [0; 0; 0; -g]

Ac = [0 1 0 0;
      0 0 0 0;
      0 0 0 1;
      0 0 0 0];

% Assumiamo (g + z_ddot)/h approx g/h per linearizzare B
Bc = [0          0;
      g/h        0;
      0          0;
      0          1/mb];

dc = [0; 0; 0; -g];

% Discretizzazione di Eulero (come descritto nell'articolo: Ak = I + Ac*dt)
Ak = eye(4) + Ac * dt;
Bk = Bc * dt;
dk = dc * dt;

%% 3. Pesi della Funzione Obiettivo (Cost Function)
% J = sum( (x - x_ref)'*S*(x - x_ref) + u'*W*u )
S = diag([1000, 10, 1000, 10]); % Pesi alti sulle posizioni, bassi sulle velocità
W = diag([1, 0.001]);           % Pesi sugli ingressi (Fz ha magnitudo alta, peso basso)

%% 4. Calcolo Limiti sugli Ingressi (Vincoli)
% Vincolo 1: -mu*h <= Delta_s <= mu*h
% Vincolo 2: -sqrt(L_max^2 - zb^2) <= Delta_s <= sqrt(L_max^2 - zb^2)
max_kinematic_ds = sqrt(L_max^2 - zb^2);

ds_min = max(-mu*h, -max_kinematic_ds);
ds_max = min(mu*h, max_kinematic_ds);

U_min = [ds_min; F_min];
U_max = [ds_max; F_max];

% Estensione dei limiti per tutto l'orizzonte
LB = repmat(U_min, N, 1);
UB = repmat(U_max, N, 1);

%% 5. Setup della Simulazione (Tracking)
T_sim = 10;                      % Tempo di simulazione [s]
steps = round(T_sim / dt);
t_vec = (0:steps-1)*dt;

% Pre-allocazione per logging dati
x_log = zeros(4, steps);
u_log = zeros(2, steps);
e_log = zeros(4, steps);
J_log = zeros(1, steps);

% Condizioni Iniziali
x0 = [0; 0; 0.6; 0]; 
x_log(:,1) = x0;

% Riferimento (Tracking Profile): Step sul displacement orizzontale, 
% Z mantenuto costante a 0.6m
x_ref = zeros(4, steps);
x_ref(1, 100:end) = 1.0;  % Muovi s in avanti di 1 metro dopo 1s
x_ref(3, :) = 0.6;        % Mantieni l'altezza a 0.6 metri

%% 6. Loop di Simulazione (MPC)
% Matrici di predizione (calcolate fuori per efficienza, essendo LTI)
Phi = zeros(4*N, 2*N);
Psi = zeros(4*N, 4);
Gamma = zeros(4*N, 1);
D_vec = repmat(dk, N, 1);

for i = 1:N
    Psi((i-1)*4+1:i*4, :) = Ak^i;
    for j = 1:i
        Phi((i-1)*4+1:i*4, (j-1)*2+1:j*2) = (Ak^(i-j)) * Bk;
    end
end

% Costruzione matrici di peso estese
S_bar = 10*kron(eye(N), S);
W_bar = 1/100*kron(eye(N), W);
H = 2 * (Phi' * S_bar * Phi + W_bar);

options = optimoptions('quadprog', 'Display', 'off'); % Silenzia quadprog

for k = 1:steps-1
    % Stato corrente e riferimento per l'orizzonte
    x_k = x_log(:, k);
    
    % Creazione vettore riferimento su orizzonte predittivo
    X_ref_vec = zeros(4*N, 1);
    for i = 1:N
        idx = min(k+i, steps); % Mantiene l'ultimo ref se eccede i boundaries
        X_ref_vec((i-1)*4+1:i*4) = x_ref(:, idx);
    end
    
    % Contributo del disturbo affine ripetuto
    Term_dist = zeros(4*N, 1);
    for i = 1:N
        for j = 1:i
            Term_dist((i-1)*4+1:i*4) = Term_dist((i-1)*4+1:i*4) + (Ak^(i-j))*dk;
        end
    end
    
    % Formulazione termine f del QP: f = 2*Phi'*S*(Psi*x0 + Term_dist - X_ref)
    f = 2 * Phi' * S_bar * (Psi * x_k + Term_dist - X_ref_vec);
    
    % Risoluzione QP 
    % (Ottimizza la sequenza di ingressi futuri U_seq)
    [U_seq, fval] = quadprog(H, f, [], [], [], [], LB, UB, [], options);
    
    % Estrai solo il primo step di controllo (Receding Horizon)
    u_opt = U_seq(1:2);
    u_log(:, k) = u_opt;
    J_log(k) = fval;
    e_log(:, k) = x_k - x_ref(:, k);
    
    % Applica ingresso al "sistema reale" (qui usiamo lo stesso modello nominale)
    x_log(:, k+1) = Ak * x_k + Bk * u_opt + dk;
end

% Ultimo step error
e_log(:, end) = x_log(:, end) - x_ref(:, end);

%% 7. Plot dei Risultati
figure('Name', 'MPC Upper-Body Control', 'Position', [100, 100, 1000, 800]);

% 1. Stato s (Spostamento orizzontale)
subplot(3, 2, 1);
plot(t_vec, x_log(1, :), 'b-', 'LineWidth', 2); hold on;
plot(t_vec, x_ref(1, :), 'r--', 'LineWidth', 1.5);
title('Spostamento Orizzontale (s)'); xlabel('Tempo [s]'); ylabel('[m]');
legend('Reale', 'Riferimento', 'Location', 'best'); grid on;

% 2. Stato z (Altezza)
subplot(3, 2, 2);
plot(t_vec, x_log(3, :), 'b-', 'LineWidth', 2); hold on;
plot(t_vec, x_ref(3, :), 'r--', 'LineWidth', 1.5);
title('Altezza CoM (z)'); xlabel('Tempo [s]'); ylabel('[m]');
legend('Reale', 'Riferimento', 'Location', 'best'); grid on;

% 3. Errore di Tracking
subplot(3, 2, 3);
plot(t_vec, e_log(1, :), 'g-', 'LineWidth', 1.5); hold on;
plot(t_vec, e_log(3, :), 'm-', 'LineWidth', 1.5);
title('Errore di Tracking'); xlabel('Tempo [s]'); ylabel('Errore [m]');
legend('Errore s', 'Errore z', 'Location', 'best'); grid on;

% 4. Ingressi: Delta s (Offset orizzontale)
subplot(3, 2, 4);
plot(t_vec(1:end-1), u_log(1, 1:end-1), 'k-', 'LineWidth', 1.5);
yline(ds_max, 'r--'); yline(ds_min, 'r--');
title('Controllo Orizzontale (\Delta s)'); xlabel('Tempo [s]'); ylabel('[m]');
legend('\Delta s', 'Limiti', 'Location', 'best'); grid on;

% 5. Ingressi: F_z (Forza verticale)
subplot(3, 2, 5);
plot(t_vec(1:end-1), u_log(2, 1:end-1), 'k-', 'LineWidth', 1.5);
yline(F_max, 'r--'); yline(F_min, 'r--');
title('Forza Inerziale Verticale (F_z)'); xlabel('Tempo [s]'); ylabel('[N]');
legend('F_z', 'Limiti', 'Location', 'best'); grid on;

% 6. Funzione Obiettivo Costo (J)
subplot(3, 2, 6);
plot(t_vec(1:end-1), J_log(1:end-1), 'c-', 'LineWidth', 1.5);
title('Andamento Cost Function (J)'); xlabel('Tempo [s]'); ylabel('Costo');
grid on;

sgtitle('Risultati Simulazione: Tracking dell''Upper-Body tramite MPC');

%% 8. Animazione del Sistema e Creazione Video
% ASSUNZIONE: IL RAGGIO DELLA RUOTA E' 0.127m (COME DA TABELLA 1 DELL'ARTICOLO).
disp('Generazione dell''animazione e salvataggio video in corso...');

% Setup VideoWriter
video_filename = 'WheeledBiped_Animation.mp4';
v = VideoWriter(video_filename, 'MPEG-4');
v.FrameRate = 25; % 25 FPS per un video fluido
open(v);

figure('Name', 'Animazione Robot', 'Position', [100, 100, 800, 400]);

r_wheel = 0.127; % Raggio ruota [m]
skip = round((1/dt) / v.FrameRate); % Sottocampionamento per adattare i 100Hz ai 25 FPS

for k = 1:skip:steps-1
    clf; hold on;
    
    % Estrazione variabili al passo k
    s_com = x_log(1, k);
    z_com = x_log(3, k);
    delta_s = u_log(1, k);
    
    % La posizione della ruota si ricava dalla definizione: Delta_s = s_com - s_wheel
    s_wheel = s_com - delta_s;
    z_wheel = r_wheel;
    
    % 1. Disegno del suolo
    plot([-1, 3], [0, 0], 'k-', 'LineWidth', 2);
    
    % 2. Disegno della ruota (cerchio)
    theta = linspace(0, 2*pi, 50);
    plot(s_wheel + r_wheel*cos(theta), z_wheel + r_wheel*sin(theta), 'k-', 'LineWidth', 2);
    plot(s_wheel, z_wheel, 'k.', 'MarkerSize', 15); % Centro della ruota
    
    % 3. Disegno dell'Upper Body (CoM)
    plot(s_com, z_com, 'bs', 'MarkerSize', 20, 'MarkerFaceColor', 'b');
    
    % 4. Disegno della gamba (collegamento Ruota - CoM)
    plot([s_wheel, s_com], [z_wheel, z_com], 'r-', 'LineWidth', 3);
    
    % 5. Disegno del riferimento desiderato (Ghost target)
    plot(x_ref(1, k), x_ref(3, k), 'gx', 'MarkerSize', 10, 'LineWidth', 2);
    
    % Impostazioni grafiche per mantenere le proporzioni costanti
    axis equal;
    xlim([-0.5, 2.0]);
    ylim([-0.1, 1.2]);
    title(sprintf('Animazione MPC - Tempo: %.2f s', t_vec(k)));
    xlabel('Spostamento Orizzontale [m]');
    ylabel('Altezza [m]');
    grid on;
    
    % Cattura del frame e scrittura nel file video
    frame = getframe(gcf);
    writeVideo(v, frame);
end

close(v);
disp(['Animazione terminata. Video salvato come: ', video_filename]);