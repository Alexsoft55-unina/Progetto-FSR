% =========================================================================
% SCRIPT DI ESTRAZIONE DATI E GENERAZIONE VIDEO SCOOTER
% =========================================================================
disp('Estrazione dati da Simulink...');

% 1. Recupero parametri fisici (devono coincidere con init.m)
params.Rw = 0.127; % Raggio ruota [m]
params.l1 = 0.45;  % Lunghezza polpaccio [m]
params.l2 = 0.45;  % Lunghezza coscia [m]
params.l3 = 0.35;  % Altezza torso [m]

% 2. Estrazione dati dal log 'out'
try
    t = out.tout; 
    data = out.sim_data; 
catch
    error('Dati non trovati. Assicurati di aver salvato i dati nel workspace come "out.sim_data".');
end

% Mux: [s, q1_ginocchio, q2_anca, q3_torso_assoluto]
s_act = data(:, 1);
q1 = data(:, 2);
q2 = data(:, 3);
q3 = data(:, 4);

% Calcoliamo l'angolo di rotazione della ruota (qw) dallo spostamento (s)
qw = s_act / params.Rw;

% Matrice Q attesa dalla funzione di animazione
Q = [qw, q1, q2, q3];

% 3. Lancio dell'animazione
nome_file = 'scooter_simulation.mp4';
disp(['Inizio generazione video: ', nome_file]);
animate_robot(t, Q, params, nome_file);


% =========================================================================
% FUNZIONE DI ANIMAZIONE (Con Cinematica Esatta da init.m)
% =========================================================================
function animate_robot(t, Q, params, filename)
    % Estrazione parametri
    Rw = params.Rw;
    l1 = params.l1;
    l2 = params.l2;
    l3 = params.l3;
    
    % Ricampionamento a 30 FPS fissi per un video fluido
    fps_target = 30;
    t_interp = (t(1) : 1/fps_target : t(end))';
    Q_interp = interp1(t, Q, t_interp, 'linear');
    
    qw = Q_interp(:, 1);
    q1 = Q_interp(:, 2); % q1 = Ginocchio
    q2 = Q_interp(:, 3); % q2 = Anca
    q3 = Q_interp(:, 4); % q3 = Torso assoluto
    
    % Setup VideoWriter
    v = VideoWriter(filename, 'MPEG-4');
    v.FrameRate = fps_target;
    v.Quality = 100;
    open(v);
    
    % Setup Figura
    fig = figure('Name', 'Scooter Robot Animation', 'Position', [100, 100, 800, 600]);
    set(fig, 'Color', 'w');
    hold on;
    grid on;
    axis equal;
    
    % Terreno
    h_ground = plot([0, 0], [0, 0], 'k', 'LineWidth', 3);
    
    % Grafica Ruota
    theta_circle = linspace(0, 2*pi, 40);
    x_circle = cos(theta_circle);
    y_circle = sin(theta_circle);
    h_wheel = plot(0, 0, 'b', 'LineWidth', 2);
    h_spoke = plot(0, 0, 'r', 'LineWidth', 2);
    
    % Grafica Link (Polpaccio, Coscia, Torso)
    h_link1 = plot(0, 0, '-o', 'Color', [0.4 0.4 0.4], 'LineWidth', 4, 'MarkerSize', 6, 'MarkerFaceColor', 'k');
    h_link2 = plot(0, 0, '-o', 'Color', [0.2 0.2 0.2], 'LineWidth', 4, 'MarkerSize', 6, 'MarkerFaceColor', 'k');
    h_link3 = plot(0, 0, '-s', 'Color', [0 0.4470 0.7410], 'LineWidth', 15); 
    
    max_height = Rw + l1 + l2 + l3 + 0.2;
    
    for i = 1:length(t_interp)
        % --- CINEMATICA (IDENTICA A init.m) ---
        
        % 0. Centro Ruota (sw)
        O0_x = Rw * qw(i);
        O0_y = Rw;
        
        % Angoli assoluti calcolati come in init.m
        phi_shank = q1(i) - q2(i) + q3(i);
        phi_thigh = q3(i) - q2(i);
        
        % 1. Ginocchio (Estremo superiore stinco, xk in init.m)
        O1_x = O0_x + l1 * sin(phi_shank);
        O1_y = O0_y + l1 * cos(phi_shank);
        
        % 2. Anca (Estremo superiore coscia, xh in init.m)
        O2_x = O1_x + l2 * sin(phi_thigh);
        O2_y = O1_y + l2 * cos(phi_thigh);
        
        % 3. Torso (x3 in init.m, qui allungato a l3 intero per grafica)
        O3_x = O2_x + l3 * sin(q3(i));
        O3_y = O2_y + l3 * cos(q3(i));
        
        % --- AGGIORNAMENTO GRAFICA ---
        set(h_wheel, 'XData', O0_x + Rw * x_circle, 'YData', O0_y + Rw * y_circle);
        set(h_spoke, 'XData', [O0_x, O0_x + Rw*cos(-qw(i))], 'YData', [O0_y, O0_y + Rw*sin(-qw(i))]);
        
        set(h_link1, 'XData', [O0_x, O1_x], 'YData', [O0_y, O1_y]);
        set(h_link2, 'XData', [O1_x, O2_x], 'YData', [O1_y, O2_y]);
        set(h_link3, 'XData', [O2_x, O3_x], 'YData', [O2_y, O3_y]);
        
        % La telecamera segue il robot
        window_width = 2.0;
        xlim([O0_x - window_width/2, O0_x + window_width/2]);
        ylim([-0.1, max_height]);
        
        % Muovi il terreno
        set(h_ground, 'XData', [O0_x - window_width, O0_x + window_width]);
        
        drawnow;
        
        % Cattura Frame e fissa le dimensioni
        frame = getframe(fig);
        [h_frame, w_frame, ~] = size(frame.cdata);
        if mod(h_frame, 2) ~= 0 || mod(w_frame, 2) ~= 0
            new_h = h_frame + mod(h_frame, 2);
            new_w = w_frame + mod(w_frame, 2);
            frame.cdata = imresize(frame.cdata, [new_h, new_w]);
        end
        
        writeVideo(v, frame);
    end
    
    close(v);
    disp(['Video salvato con successo: ', filename]);
end