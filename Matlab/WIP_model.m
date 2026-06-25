%% Table 1. Parameters of the decoupled model.

% --- Parametri Fisici Costanti ---
m_w = 3.5;      % Mass of the wheel [kg]
I_w = 0.1;      % Moment of inertia of the wheel [kg*m^2]
r = 0.127;      % Radius of the wheel [m]
d = 0.63;       % Distance between two wheels [m]

% --- Variabili di Stato e Ingressi ---
% Poiché questi parametri non hanno un valore fisso nella tabella (indicati con '/'),
% vengono inizializzati come variabili simboliche (richiede il Symbolic Math Toolbox).
syms l theta phi s tau_l real

% Descrizione delle variabili simboliche:
% l     : Length of the pendulum
% theta : Tilt angle of the pendulum
% phi   : Yaw angle of the VL-WIP (utilizzando 'phi' per la lettera greca φ)
% s     : Displacement of the VL-WIP
% tau_l : Torque about the left wheel

%% Table 1. Cont. - Parametri aggiuntivi del modello

% --- Variabili di Stato e Ingressi (aggiuntive) ---
syms tau_r q_1 q_2 q_3 real

% Descrizione delle variabili simboliche:
% tau_r : Torque about the right wheel
% q_1   : Angle of the knee joint
% q_2   : Angle of the hip joint
% q_3   : Pitch angle of the torso

% --- Parametri Fisici Costanti ---
m_b = 73;       % Mass of the upper body [kg]
I_z = 3.3;      % Moment of inertia about the z-axis [kg*m^2]
m_1 = 1.2;      % Mass of the shank [kg]
m_2 = 5.3;      % Mass of the thigh [kg]
m_3 = 60;       % Mass of the torso [kg]
l_1 = 0.45;     % Length of the shank [m]
l_2 = 0.45;     % Length of the thigh [m]
l_3 = 0.35;     % Height of the torso [m]

% --- Parametri Derivati / Equazioni ---
% Nota: l'espressione per I_y utilizza 'l' (Length of the pendulum), 
% che è stata dichiarata nella prima parte dello script. 
% Se 'l' è una variabile simbolica, anche I_y sarà un'espressione simbolica.
I_y = (1/3) * m_b * l^2;  % Moment of inertia about the y-axis

% L'angolo q_0 è definito in funzione delle altre variabili di giunto.
q_0 = q_1 - q_2 + q_3;    % Angle of the ankle joint

%% Equazioni delle Matrici Linearizzate e Parametri Intermedi

% --- Costante aggiuntiva ---
g = 9.81; % Accelerazione di gravità [m/s^2] (assunta standard)

% --- Calcolo del Denominatore Comune ---
% I parametri a_1, a_2, b_1 e b_2 condividono lo stesso denominatore.
% Lo calcolo una volta sola per mantenere il codice pulito ed efficiente.
den_comune = 2*I_w*(I_y + m_b*l^2) + (2*l^2*m_b*m_w + I_y*(m_b + 2*m_w)*r^2);

% --- Calcolo dei Coefficienti ---
a_1 = - (g * l^2 * m_b^2 * r^2) / den_comune;

a_2 = (g * l * m_b * (2*I_w + (m_b + 2*m_w)*r^2)) / den_comune;

b_1 = (r * (I_y + l*m_b*(l + r))) / den_comune;

b_2 = - (2*I_w + r*(l*m_b + (m_b + 2*m_w)*r)) / den_comune;

b_3 = (d * r) / (2*I_z*r^2 + d^2*(I_w + m_w*r^2));

% --- Costruzione delle Matrici del Sistema Linearizzato ---

% Matrice A(l) (6x6)
A = [0,   0,   0,   1,   0,   0;
     0,   0,   0,   0,   1,   0;
     0,   0,   0,   0,   0,   1;
     0, a_1,   0,   0,   0,   0;
     0, a_2,   0,   0,   0,   0;
     0,   0,   0,   0,   0,   0];

% Matrice B(l) (6x2)
B = [  0,    0;
       0,    0;
       0,    0;
     b_1,  b_1;
     b_2,  b_2;
     b_3, -b_3];

% Vettore degli ingressi U (2x1)
U = [tau_l; 
     tau_r];

% 1. Definizione del Punto Operativo Nominale
% Fissiamo una lunghezza nominale del pendolo per valutare A(l) e B(l)
l_nom = 0.5; % [m] Sostituisci con il valore di progetto reale

% Creiamo la struttura parametri per la simulazione non lineare
params.m_b = m_b; params.m_w = m_w; params.I_w = I_w;
params.l = l_nom; params.r = r;     params.d = d; 
params.I_z = I_z;

% Valutazione numerica di I_y e delle matrici A e B
I_y_num = double(subs(I_y, l, l_nom));
params.I_y = I_y_num;

A_num = double(subs(A, l, l_nom));
B_num = double(subs(B, l, l_nom));

%% 2. Analisi di Controllabilità
Co = ctrb(A_num, B_num);
if rank(Co) < size(A_num, 1)
    error('Il sistema linearizzato non è completamente controllabile.');
else
    disp('Sistema controllabile. Procedo con la sintesi LQR...');
end

%% 3. Sintesi del Controllore LQR
% Stati del sistema: X = [s; theta; phi; s_dot; theta_dot; phi_dot]

% Matrice Q (6x6) - Pesi sugli stati
% Penalizziamo in modo aggressivo l'errore sull'angolo theta e la sua derivata 
% per mantenere l'equilibrio del pendolo inverso.
Q = diag([10, 1000, 10, 1, 100, 1]); 

% Matrice R (2x2) - Pesi sugli ingressi (sforzo di controllo tau_l, tau_r)
% Un valore basso indica che abbiamo a disposizione motori performanti
R = diag([0.1, 0.1]); 

% Calcolo della matrice dei guadagni K
[K, S, e] = lqr(A_num, B_num, Q, R);
disp('Matrice dei guadagni K calcolata:');
disp(K);

%% 4. Simulazione sul Plant Non Lineare (ode45) con Inseguimento
% Condizioni iniziali X0: Partiamo dall'origine, fermi e in equilibrio
X0 = [0; 0; 0; 0; 0; 0]; 

% Vettore di stato desiderato X_des:
% Supponiamo di voler avanzare di 1 metro (s=1) e ruotare di 45 gradi
s_des = 1.0;              % Posizione desiderata [m]
phi_des = deg2rad(10);    % Imbardata desiderata [rad]
X_des = [s_des; 0; phi_des; 0; 0; 0];

% Definizione della matrice di ingresso per il riferimento Bv
% Essendo X_des un equilibrio naturale, la precompensazione ottimale è Bv = K
Bv = K;

% Orizzonte di simulazione [secondi]
% Aumentato leggermente per permettere al sistema di assestarsi nella nuova posizione
t_span = [0 30]; 

% Funzione anonima (wrapper) aggiornata per includere X_des e Bv
dinamica_chiusa = @(t, X) dinamica_non_lineare_wrapper(t, X, X_des, K, Bv, params);

% Risoluzione dell'equazione differenziale
options = odeset('RelTol', 1e-4, 'AbsTol', 1e-6);
[t_sim, X_sim] = ode45(dinamica_chiusa, t_span, X0, options);


%% 5. Plot dei Risultati
figure('Name', 'Risultati Simulazione LQR VL-WIP', 'Color', 'w');

subplot(3,1,1);
plot(t_sim, X_sim(:,1), 'b', 'LineWidth', 1.5);
ylabel('Posizione s [m]'); 
title('Spostamento Lineare'); 
grid on;

subplot(3,1,2);
plot(t_sim, rad2deg(X_sim(:,2)), 'r', 'LineWidth', 1.5);
ylabel('Angolo \theta [deg]'); 
title('Angolo di Tilt (Pendolo)'); 
grid on;

subplot(3,1,3);
plot(t_sim, rad2deg(X_sim(:,3)), 'k', 'LineWidth', 1.5);
ylabel('Angolo \phi [deg]'); 
xlabel('Tempo [s]'); 
title('Angolo di Imbardata (Yaw)'); 
grid on;


%% Video

%% 6. Animazione 3D e Salvataggio Video (WIP_Simulation.mp4)

disp('Generazione del video in corso...');

% Creazione e configurazione dell'oggetto VideoWriter
video_filename = 'WIP_Simulation.mp4';
v = VideoWriter(video_filename, 'MPEG-4');
v.FrameRate = 30; % Fotogrammi al secondo
open(v);

% Setup della figura per l'animazione
fig_anim = figure('Name', 'Animazione VL-WIP', 'Color', 'w', 'Position', [100, 100, 800, 600]);
hold on; grid on; axis equal;
view(3); % Vista 3D

% Imposta i limiti degli assi in base allo spostamento massimo previsto
xlim([-1 1]); 
ylim([-1 1]); 
zlim([0 1.2]);
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Simulazione Dinamica VL-WIP');

% Parametri geometrici per il disegno
r = params.r;
d = params.d;
l_anim = params.l; % Lunghezza del pendolo (fissa per questa simulazione)

% Inizializzazione degli oggetti grafici (Handles)
% Usiamo linee spesse per simulare l'asse e il pendolo, e dei cerchi per le ruote
h_axle = plot3([0 0], [0 0], [0 0], 'k-', 'LineWidth', 6);
h_rod  = plot3([0 0], [0 0], [0 0], 'b-', 'LineWidth', 4);
h_mass = plot3(0, 0, 0, 'ko', 'MarkerSize', 15, 'MarkerFaceColor', 'r');
h_wheelL = plot3(0, 0, 0, 'k-', 'LineWidth', 3);
h_wheelR = plot3(0, 0, 0, 'k-', 'LineWidth', 3);
h_path = plot3(0, 0, 0, 'g--', 'LineWidth', 1.5); % Traccia il percorso

% Per evitare che l'animazione sia troppo lenta, campioniamo a ~30 fps
% ode45 genera step di tempo variabili, quindi interpoliamo
t_video = 0:(1/v.FrameRate):t_sim(end);
X_video = interp1(t_sim, X_sim, t_video);

path_x = zeros(1, length(t_video));
path_y = zeros(1, length(t_video));

% Ciclo di animazione
for i = 1:length(t_video)
    % Estrazione stati correnti
    s     = X_video(i, 1);
    theta = X_video(i, 2);
    phi   = X_video(i, 3);
    
    % 1. Cinematica della base (Centro dell'asse)
    % Assumiamo che s sia l'avanzamento lungo la direzione phi
    x_c = s * cos(phi);
    y_c = s * sin(phi);
    z_c = r;
    
    path_x(i) = x_c;
    path_y(i) = y_c;
    
    % 2. Posizione delle ruote
    % Direzione trasversale (perpendicolare all'avanzamento)
    dx_trans = (d/2) * -sin(phi);
    dy_trans = (d/2) * cos(phi);
    
    x_L = x_c + dx_trans; y_L = y_c + dy_trans;
    x_R = x_c - dx_trans; y_R = y_c - dy_trans;
    
    % 3. Posizione del baricentro del pendolo (Massa superiore)
    % Inclinazione theta nel piano dell'avanzamento (phi)
    x_p = x_c + l_anim * sin(theta) * cos(phi);
    y_p = y_c + l_anim * sin(theta) * sin(phi);
    z_p = r + l_anim * cos(theta);
    
    % 4. Aggiornamento delle coordinate degli oggetti grafici
    set(h_axle, 'XData', [x_L x_R], 'YData', [y_L y_R], 'ZData', [r r]);
    set(h_rod, 'XData', [x_c x_p], 'YData', [y_c y_p], 'ZData', [z_c z_p]);
    set(h_mass, 'XData', x_p, 'YData', y_p, 'ZData', z_p);
    set(h_path, 'XData', path_x(1:i), 'YData', path_y(1:i), 'ZData', repmat(r, 1, i));
    
    % Disegno rudimentale delle ruote (cerchi nel piano di rotolamento)
    angoli = linspace(0, 2*pi, 20);
    % Ruota sinistra
    wxL = x_L + r * cos(angoli) * cos(phi);
    wyL = y_L + r * cos(angoli) * sin(phi);
    wzL = r + r * sin(angoli);
    set(h_wheelL, 'XData', wxL, 'YData', wyL, 'ZData', wzL);
    % Ruota destra
    wxR = x_R + r * cos(angoli) * cos(phi);
    wyR = y_R + r * cos(angoli) * sin(phi);
    wzR = r + r * sin(angoli);
    set(h_wheelR, 'XData', wxR, 'YData', wyR, 'ZData', wzR);
    
    % Forziamo MATLAB a disegnare il frame
    drawnow;
    
    % Catturiamo il frame e lo scriviamo nel video
    frame = getframe(fig_anim);
    writeVideo(v, frame);
end

% Chiusura sicura del file video
close(v);
disp(['Video salvato con successo: ', fullfile(pwd, video_filename)]);

function dX = dinamica_non_lineare_wrapper(t, X, X_des, K, Bv, params)
    % 1. Calcolo dell'azione di controllo con Tracking
    % Legge LQR modificata: U = -K*X + Bv*X_des
    tau_w = -K * X + Bv * X_des; 
    
    % (Opzionale) Saturazione per simulare i limiti fisici dei motori
    % Utile quando si danno "gradini" di riferimento ampi
    % tau_max = 10; % [Nm]
    % tau_w = max(min(tau_w, tau_max), -tau_max);
    
    % 2. Richiamo la funzione della dinamica non lineare reale
    dX = dinamica_non_lineare(t, X, tau_w, params);
end
function dX = dinamica_non_lineare(t, X, tau_w, params)
    % Vettore di stato X = [s; theta; phi; s_dot; theta_dot; phi_dot]
    % (Nota: uso 'phi_yaw' per la coordinata generalizzata e 'theta' per il pendolo)
    
    theta = X(2);
    theta_dot = X(5);
    
    % Estrazione parametri dalla struct 'params'
    mb = params.m_b; mw = params.m_w; Iw = params.I_w;
    l = params.l; r = params.r; d = params.d; 
    Iy = params.I_y; Iz = params.I_z; g = 9.81;

    % --- 1. Matrice di Inerzia Generalizzata M(phi) ---
    M11 = mb + 2*mw + 2*(Iw/r^2);
    M12 = mb*l*cos(theta);
    M22 = mb*l^2 + Iy;
    M33 = (d^2*mw)/2 + (d^2*Iw)/(2*r^2) + Iz;
    
    M = [M11,   M12,   0;
         M12,   M22,   0;
         0,     0,     M33];

    % --- 2. Vettore di Coriolis, Centripeta e Gravità C(phi, phi_dot) ---
    % Nota: L'immagine mostra C come un vettore colonna 3x1
    C = [ -mb * l * (theta_dot^2) * sin(theta);
          -mb * g * l * sin(theta);
           0 ];

    % --- 3. Matrice di Ingresso B (Non lineare) ---
    % Attenzione: questa matrice B è diversa dalla B(l) linearizzata!
    B_nl = [ 1/r,      1/r;
            -1,       -1;
             d/(2*r), -d/(2*r) ];

    % --- 4. Calcolo delle accelerazioni (phi_ddot) ---
    % In MATLAB è computazionalmente più efficiente e stabile usare 
    % l'operatore 'mldivide' (\) invece di calcolare l'inversa inv(M)
    phi_ddot = M \ (B_nl * tau_w - C);

    % --- 5. Aggiornamento delle derivate dello stato ---
    % dX = [velocità; accelerazioni]
    dX = [ X(4:6); 
           phi_ddot ];
end