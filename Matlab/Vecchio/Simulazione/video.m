% =========================================================================
% SCRIPT DI CONFIGURAZIONE ED ESTRAZIONE DATI SIMULINK
% =========================================================================

% 1. Parametri del robot
params.Rw = robot_params.Rw; % Raggio ruota [m]
params.l1 = robot_params.l1;  % Lunghezza link 1 [m]
params.l2 = robot_params.l2;  % Lunghezza link 2 [m]
params.l3 = robot_params.l3;  % Lunghezza/Altezza link 3 (corpo) [m]

% Estrazione del vettore tempo
t = out.tout; 

% Estrazione della matrice dei giunti Q
Q_raw = out.q.Data; 

% Condizionamento della matrice Q per garantire il formato N x 4
if ndims(Q_raw) == 3
    % Permuta e comprime per ottenere N x 4 (tipico di Simulink)
    Q = squeeze(Q_raw)'; 
elseif size(Q_raw, 2) ~= 4 && size(Q_raw, 1) == 4
    % Se è salvata come 4 x N, la trasponiamo
    Q = Q_raw';
else
    % Se è già N x 4, la teniamo così
    Q = Q_raw;
end

%% === LA CHIAMATA MANCANTE ===
% Questo blocco dice a MATLAB di eseguire effettivamente l'animazione
disp('Inizio generazione video dai dati Simulink...');
animate_robot(t, Q, params, 'robot_simulink.mp4');


% =========================================================================
% FUNZIONE DI ANIMAZIONE AGGIORNATA E CORRETTA
% =========================================================================
function animate_robot(t, Q, params, filename)
    % Estrazione parametri
    Rw = params.Rw;
    l1 = params.l1;
    l2 = params.l2;
    l3 = params.l3;
    
    % 1. RISOLUZIONE PROBLEMA FRAMERATE (Ricampionamento a 30 FPS fissi)
    fps_target = 30;
    t_interp = (t(1) : 1/fps_target : t(end))';
    
    % Interpolazione lineare degli stati per i nuovi istanti di tempo
    Q_interp = interp1(t, Q, t_interp, 'linear');
    
    % Estrazione coordinate articolari ricampionate
    qw = Q_interp(:, 1);
    q1 = Q_interp(:, 2);
    q2 = Q_interp(:, 3);
    q3 = Q_interp(:, 4);
    
    % Configurazione VideoWriter con FrameRate fisso e sicuro
    v = VideoWriter(filename, 'MPEG-4');
    v.FrameRate = fps_target;
    v.Quality = 100;
    open(v);
    
    % Setup della figura (forziamo dimensioni pari in partenza)
    fig = figure('Name', 'Animazione Robot Sagittale', 'Position', [100, 100, 900, 600]);
    set(fig, 'Color', 'w'); % Sfondo bianco per una resa migliore
    hold on;
    grid on;
    axis equal;
    
    % Inizializzazione handle grafici
    h_ground = plot([0, 0], [0, 0], 'k', 'LineWidth', 2);
    
    % Cerchio per disegnare la ruota
    theta_circle = linspace(0, 2*pi, 40);
    x_circle = cos(theta_circle);
    y_circle = sin(theta_circle);
    h_wheel = plot(0, 0, 'b', 'LineWidth', 2);
    h_spoke = plot(0, 0, 'r', 'LineWidth', 2);
    
    % Linee per i link
    h_link1 = plot(0, 0, '-o', 'Color', [0 0.4470 0.7410], 'LineWidth', 4, 'MarkerSize', 6, 'MarkerFaceColor', 'k');
    h_link2 = plot(0, 0, '-o', 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 4, 'MarkerSize', 6, 'MarkerFaceColor', 'k');
    h_link3 = plot(0, 0, '-s', 'Color', [0.9290 0.6940 0.1250], 'LineWidth', 8); 
    
    % Calcolo altezza massima per limitare e stabilizzare gli assi
    max_height = Rw + l1 + l2 + l3;
    
    disp(['Generazione di ', num2str(length(t_interp)), ' frame a 30 FPS in corso...']);
    
    % Forza l'apertura e la visualizzazione della figura in primo piano
    figure(fig);
    drawnow;
    
    for i = 1:length(t_interp)
        % --- CINEMATICA ---
        sw = Rw * qw(i);
        
        % Centro della ruota (Giunto 0)
        O0_x = sw;
        O0_y = Rw;
        
        % Giunto 1 (Tra link 1 e link 2)
        O1_x = O0_x + l1 * cos(q1(i));
        O1_y = O0_y + l1 * sin(q1(i));
        
        % Giunto 2 (Tra link 2 e link 3)
        O2_x = O1_x + l2 * cos(q1(i) + q2(i));
        O2_y = O1_y + l2 * sin(q1(i) + q2(i));
        
        % Estremità del link 3 (il corpo)
        O3_x = O2_x + l3 * sin(q3(i));
        O3_y = O2_y + l3 * cos(q3(i));
        
        % --- AGGIORNAMENTO GRAFICA ---
        
        % Aggiorna ruota e raggio
        set(h_wheel, 'XData', O0_x + Rw * x_circle, 'YData', O0_y + Rw * y_circle);
        set(h_spoke, 'XData', [O0_x, O0_x + Rw*cos(-qw(i))], 'YData', [O0_y, O0_y + Rw*sin(-qw(i))]);
        
        % Aggiorna link
        set(h_link1, 'XData', [O0_x, O1_x], 'YData', [O0_y, O1_y]);
        set(h_link2, 'XData', [O1_x, O2_x], 'YData', [O1_y, O2_y]);
        set(h_link3, 'XData', [O2_x, O3_x], 'YData', [O2_y, O3_y]);
        
        % Finestra dinamica della telecamera
        window_width = max_height * 2.5;
        xlim([O0_x - window_width/2, O0_x + window_width/2]);
        ylim([-0.1, max_height + 0.5]);
        
        % Aggiorna il terreno
        set(h_ground, 'XData', [O0_x - window_width, O0_x + window_width]);
        
        drawnow;
        
        % --- CATTURA FRAME E RISOLUZIONE DIMENSIONI DISPARI ---
        frame = getframe(fig);
        
        % Ottieni dimensioni dell'immagine catturata
        [h_frame, w_frame, ~] = size(frame.cdata);
        
        % Se altezza o larghezza sono dispari, ridimensiona forzando a numero pari
        if mod(h_frame, 2) ~= 0 || mod(w_frame, 2) ~= 0
            new_h = h_frame + mod(h_frame, 2);
            new_w = w_frame + mod(w_frame, 2);
            frame.cdata = imresize(frame.cdata, [new_h, new_w]);
        end
        
        writeVideo(v, frame);
    end
    
    close(v);
    disp(['Video salvato con successo come: ', filename]);
end