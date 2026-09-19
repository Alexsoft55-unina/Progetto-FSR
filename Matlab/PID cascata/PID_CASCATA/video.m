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
    Rw = params.Rw; l1 = params.l1; l2 = params.l2; l3 = params.l3;
    
    % Ricampionamento
    fps_target = 30;
    t_interp = (t(1) : 1/fps_target : t(end))';
    Q_interp = interp1(t, Q, t_interp, 'linear');
    
    % Allineamento convenzione: Q = [q1, qw, q2, q3]
    q1 = Q_interp(:, 1); 
    qw = Q_interp(:, 2); 
    q2 = Q_interp(:, 3);
    q3 = Q_interp(:, 4);
    
    v = VideoWriter(filename, 'MPEG-4'); v.FrameRate = fps_target; open(v);
    fig = figure('Name', 'Animazione Robot', 'Position', [100, 100, 900, 600], 'Color', 'w');
    hold on; grid on; axis equal;
    
    % Handle grafici
    h_ground = plot([0, 0], [0, 0], 'k', 'LineWidth', 2);
    h_wheel = plot(0, 0, 'b', 'LineWidth', 2);
    h_link1 = plot(0, 0, '-o', 'Color', [0 0.447 0.741], 'LineWidth', 4);
    h_link2 = plot(0, 0, '-o', 'Color', [0.850 0.325 0.098], 'LineWidth', 4);
    h_link3 = plot(0, 0, '-s', 'Color', [0.929 0.694 0.125], 'LineWidth', 8); 
    
    for i = 1:length(t_interp)
        % Cinematica coerente con il modello lagrangiano:
        % Base ruota traslata da qw
        sw = Rw * qw(i);
        O0 = [sw; Rw];
        
        % Giunto 1 (q1)
        O1 = O0 + [l1*cos(q1(i)); l1*sin(q1(i))];
        
        % Giunto 2 (q1 + q2)
        O2 = O1 + [l2*cos(q1(i)+q2(i)); l2*sin(q1(i)+q2(i))];
        
        % Estremità Torso (q3)
        % Nota: Nel tuo script usavi sin(q3) per x e cos(q3) per y
        O3 = O2 + [l3*sin(q3(i)); l3*cos(q3(i))];
        
        % Aggiornamento
        theta = linspace(0, 2*pi, 40);
        set(h_wheel, 'XData', O0(1) + Rw*cos(theta), 'YData', O0(2) + Rw*sin(theta));
        set(h_link1, 'XData', [O0(1), O1(1)], 'YData', [O0(2), O1(2)]);
        set(h_link2, 'XData', [O1(1), O2(1)], 'YData', [O1(2), O2(2)]);
        set(h_link3, 'XData', [O2(1), O3(1)], 'YData', [O2(2), O3(2)]);
        
        xlim([O0(1)-1.5, O0(1)+1.5]); ylim([-0.1, 1.5]);
        set(h_ground, 'XData', [O0(1)-5, O0(1)+5]);
        
        drawnow; writeVideo(v, getframe(fig));
    end
    close(v);
end