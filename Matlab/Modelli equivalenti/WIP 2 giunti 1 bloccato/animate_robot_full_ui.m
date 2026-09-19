function animate_robot_full_ui(out)
    % ANIMATE_ROBOT_FULL_UI Riproduzione grafica interattiva della dinamica
    % con plot dinamici sincronizzati. Gestisce segnali continui e discreti.
    
    %% 1. Estrazione e Parsing dei Dati da Simulink
    try
        % Vettore tempo globale di fallback
        t_base = out.tout;
        
        % --- Estrazione Q ---
        if isprop(out, 'q') && isa(out.q, 'timeseries')
            q_data = out.q.Data;
            t_q = out.q.Time;
        elseif isprop(out, 'q')
            q_data = out.q; 
            t_q = t_base;
        else
            error('Variabile "q" non trovata nell''output.');
        end
        if size(q_data, 2) > size(q_data, 1), q_data = q_data'; end
        if length(t_q) ~= size(q_data, 1), t_q = linspace(t_q(1), t_q(end), size(q_data,1))'; end
        
        % --- Estrazione Pendolo Equivalente ---
        if isprop(out, 'pend_eq') && isa(out.pend_eq, 'timeseries')
            pend_eq_data = out.pend_eq.Data;
            t_pend = out.pend_eq.Time;
        elseif isprop(out, 'pend_eq')
            pend_eq_data = out.pend_eq;
            t_pend = t_base;
        else
            error('Variabile "pend_eq" non trovata nell''output.');
        end
        if size(pend_eq_data, 2) > size(pend_eq_data, 1), pend_eq_data = pend_eq_data'; end
        if length(t_pend) ~= size(pend_eq_data, 1), t_pend = linspace(t_pend(1), t_pend(end), size(pend_eq_data,1))'; end
        
        % --- Estrazione Coppia (tau o tau_wheel) ---
        if isprop(out, 'tau') && isa(out.tau, 'timeseries')
            tau_data = out.tau.Data;
            t_tau = out.tau.Time;
        elseif isprop(out, 'tau')
            tau_data = out.tau;
            t_tau = t_base;
        elseif isprop(out, 'tau_wheel') && isa(out.tau_wheel, 'timeseries')
            tau_data = out.tau_wheel.Data;
            t_tau = out.tau_wheel.Time;
        else
            error('Variabile "tau" o "tau_wheel" non trovata nell''output.');
        end
        
        % Formattazione matrice coppia
        if size(tau_data, 2) > size(tau_data, 1) && size(tau_data, 1) <= 3
            tau_data = tau_data'; 
        end
        if size(tau_data, 2) >= 2
            tau_wheel = tau_data(:, 2);
        else
            tau_wheel = tau_data(:, 1);
        end
        if length(t_tau) ~= length(tau_wheel)
            t_tau = linspace(t_tau(1), t_tau(end), length(tau_wheel))'; 
        end
        
    catch ME
        fprintf('Errore nell''estrazione dei dati. Verifica la formattazione.\n');
        rethrow(ME);
    end

    % Assegnazione Variabili di Stato
    q1 = q_data(:, 1);
    qw = q_data(:, 2);
    q2 = q_data(:, 3);
    l_eq     = pend_eq_data(:, 1);
    theta_eq = pend_eq_data(:, 2);

    %% 2. Parametri Geometrici
    Rw = 0.127; % [m]
    l1 = 0.45;  % [m]
    l2 = 0.45;  % [m]

    %% 3. Interpolazione a Framerate Costante (60 FPS)
    fps = 60; 
    dt_anim = 1/fps;
    
    % Creiamo il vettore temporale dell'animazione basato sui limiti effettivi
    t_start = max([t_q(1), t_pend(1), t_tau(1)]);
    t_end = min([t_q(end), t_pend(end), t_tau(end)]);
    t_anim = t_start:dt_anim:t_end;
    num_frames = length(t_anim);

    % Interpoliamo ogni segnale sul proprio vettore temporale indipendente
    % Usiamo 'extrap' per prevenire eventuali piccolissime divergenze ai margini
    q1_int = interp1(t_q, q1, t_anim, 'linear', 'extrap');
    qw_int = interp1(t_q, qw, t_anim, 'linear', 'extrap');
    q2_int = interp1(t_q, q2, t_anim, 'linear', 'extrap');
    leq_int = interp1(t_pend, l_eq, t_anim, 'linear', 'extrap');
    teq_int = interp1(t_pend, theta_eq, t_anim, 'linear', 'extrap');
    
    % Per un segnale di controllo (Zero-Order Hold), l'interpolazione a gradino 
    % ('previous') rappresenta meglio il reale comportamento discreto, ma 'linear' 
    % evita grafiche "spezzate" in bassa risoluzione. Scegliamo 'linear' per fluidità.
    tau_w_int = interp1(t_tau, tau_wheel, t_anim, 'linear', 'extrap');

    %% 4. Pre-calcolo Vettorizzato della Cinematica
    xw_all = Rw * qw_int;
    yw_all = repmat(Rw, 1, num_frames);
    
    xk_all = xw_all + l1 * sin(q1_int);
    yk_all = yw_all + l1 * cos(q1_int);
    
    xh_all = xk_all + l2 * sin(q2_int);
    yh_all = yk_all + l2 * cos(q2_int);
    
    x_eq_all = xw_all + leq_int .* sin(teq_int);
    y_eq_all = yw_all + leq_int .* cos(teq_int);

    theta_circle = linspace(0, 2*pi, 50);
    x_circle_base = Rw * cos(theta_circle);
    y_circle_base = Rw * sin(theta_circle);

    %% 5. Setup dell'Interfaccia Grafica e Assi
    fig = figure('Name', 'Animazione UI e Plot Dinamici - Doppio Pendolo', ...
                 'Color', 'w', 'Position', [50 50 1400 900]);
             
    % --- Asse per l'Animazione (Top Half) ---
    ax_anim = axes(fig, 'Position', [0.05 0.55 0.9 0.4]);
    hold(ax_anim, 'on'); grid(ax_anim, 'on'); axis(ax_anim, 'equal');
    
    y_lim = [-0.1, l1 + l2 + 0.4];
    ylim(ax_anim, y_lim);
    xlabel(ax_anim, 'Posizione X [m]', 'FontWeight', 'bold');
    ylabel(ax_anim, 'Posizione Y [m]', 'FontWeight', 'bold');
    title(ax_anim, 'Dinamica del Robot e Modello Equivalente', 'FontSize', 12);

    % --- Assi per i Plot Dinamici (Bottom Half, 4 colonne) ---
    ax_q = axes(fig, 'Position', [0.04 0.2 0.2 0.25]);
    hold(ax_q, 'on'); grid(ax_q, 'on');
    title(ax_q, 'Angoli Stinco (q1) e Coscia (q2)', 'FontSize', 10);
    xlabel(ax_q, 'Tempo [s]'); ylabel(ax_q, 'Angolo [rad]');
    xlim(ax_q, [t_anim(1) t_anim(end)]);

    ax_qw = axes(fig, 'Position', [0.28 0.2 0.2 0.25]);
    hold(ax_qw, 'on'); grid(ax_qw, 'on');
    title(ax_qw, 'Rotazione Ruota (qw)', 'FontSize', 10);
    xlabel(ax_qw, 'Tempo [s]'); ylabel(ax_qw, 'Angolo [rad]');
    xlim(ax_qw, [t_anim(1) t_anim(end)]);

    ax_eq = axes(fig, 'Position', [0.52 0.2 0.2 0.25]);
    hold(ax_eq, 'on'); grid(ax_eq, 'on');
    title(ax_eq, 'Angolo Modello Eq. (\theta_{eq})', 'FontSize', 10);
    xlabel(ax_eq, 'Tempo [s]'); ylabel(ax_eq, 'Angolo [rad]');
    xlim(ax_eq, [t_anim(1) t_anim(end)]);

    ax_tau = axes(fig, 'Position', [0.76 0.2 0.2 0.25]);
    hold(ax_tau, 'on'); grid(ax_tau, 'on');
    title(ax_tau, 'Coppia Motore (\tau_{wheel})', 'FontSize', 10);
    xlabel(ax_tau, 'Tempo [s]'); ylabel(ax_tau, 'Coppia [Nm]');
    xlim(ax_tau, [t_anim(1) t_anim(end)]);

    %% 6. Creazione degli Oggetti Grafici
    % --- Elementi Animazione ---
    c_wheel = [0.2 0.2 0.2];
    c_link1 = [0.0 0.45 0.74];  
    c_link2 = [0.85 0.33 0.10]; 
    c_eq    = [0.47 0.67 0.19]; 
    c_tau   = [0.63 0.08 0.18]; 
    
    plot(ax_anim, [-100 100], [0 0], 'k', 'LineWidth', 2); % Terreno
    h_wheel_center = plot(ax_anim, 0, Rw, 'o', 'MarkerSize', 6, 'MarkerFaceColor', c_wheel, 'MarkerEdgeColor', 'k');
    h_wheel_circle = plot(ax_anim, 0, 0, 'Color', c_wheel, 'LineWidth', 2); 
    h_wheel_line   = plot(ax_anim, [0 0], [0 Rw], 'Color', c_wheel, 'LineWidth', 2);
    
    h_link1 = plot(ax_anim, [0 0], [0 0], 'Color', c_link1, 'LineWidth', 4);
    h_link2 = plot(ax_anim, [0 0], [0 0], 'Color', c_link2, 'LineWidth', 4);
    
    h_knee = plot(ax_anim, 0, 0, 'ko', 'MarkerFaceColor', c_link1, 'MarkerSize', 8);
    h_hip  = plot(ax_anim, 0, 0, 'ko', 'MarkerFaceColor', c_link2, 'MarkerSize', 10);
    h_pend_eq_line = plot(ax_anim, [0 0], [0 0], '--', 'Color', c_eq, 'LineWidth', 2.5);
    h_pend_eq_mass = plot(ax_anim, 0, 0, 's', 'MarkerFaceColor', c_eq, 'MarkerEdgeColor', 'k', 'MarkerSize', 12);
    legend(ax_anim, [h_link1, h_link2, h_pend_eq_line], ...
           {'Stinco (Link 1)', 'Coscia (Link 2)', 'Pendolo Equivalente'}, ...
           'Location', 'northeast');

    % --- Elementi Plot Dinamici (Sfondo statico) ---
    plot(ax_q, t_anim, q1_int, 'Color', [c_link1, 0.4], 'LineWidth', 1.5, 'DisplayName', 'q1 (Stinco)');
    plot(ax_q, t_anim, q2_int, 'Color', [c_link2, 0.4], 'LineWidth', 1.5, 'DisplayName', 'q2 (Coscia)');
    legend(ax_q, 'Location', 'best');
    
    plot(ax_qw, t_anim, qw_int, 'Color', [c_wheel, 0.4], 'LineWidth', 1.5);
    plot(ax_eq, t_anim, teq_int, 'Color', [c_eq, 0.4], 'LineWidth', 1.5);
    plot(ax_tau, t_anim, tau_w_int, 'Color', [c_tau, 0.4], 'LineWidth', 1.5);

    % --- Indicatori Dinamici (Pallini mobili) ---
    h_dot_q1  = plot(ax_q, t_anim(1), q1_int(1), 'o', 'MarkerFaceColor', c_link1, 'MarkerEdgeColor', 'k', 'MarkerSize', 7);
    h_dot_q2  = plot(ax_q, t_anim(1), q2_int(1), 'o', 'MarkerFaceColor', c_link2, 'MarkerEdgeColor', 'k', 'MarkerSize', 7);
    h_dot_qw  = plot(ax_qw, t_anim(1), qw_int(1), 'o', 'MarkerFaceColor', c_wheel, 'MarkerEdgeColor', 'k', 'MarkerSize', 7);
    h_dot_eq  = plot(ax_eq, t_anim(1), teq_int(1), 'o', 'MarkerFaceColor', c_eq, 'MarkerEdgeColor', 'k', 'MarkerSize', 7);
    h_dot_tau = plot(ax_tau, t_anim(1), tau_w_int(1), 'o', 'MarkerFaceColor', c_tau, 'MarkerEdgeColor', 'k', 'MarkerSize', 7);

    % --- Linee temporali verticali ---
    h_tbar_q   = plot(ax_q,  [t_anim(1) t_anim(1)], ylim(ax_q),  'k--', 'LineWidth', 1);
    h_tbar_qw  = plot(ax_qw, [t_anim(1) t_anim(1)], ylim(ax_qw), 'k--', 'LineWidth', 1);
    h_tbar_eq  = plot(ax_eq, [t_anim(1) t_anim(1)], ylim(ax_eq), 'k--', 'LineWidth', 1);
    h_tbar_tau = plot(ax_tau, [t_anim(1) t_anim(1)], ylim(ax_tau), 'k--', 'LineWidth', 1);

    %% 7. Creazione Componenti UI
    current_idx = 1;
    is_playing = false;

    slider = uicontrol('Style', 'slider', ...
        'Min', 1, 'Max', num_frames, 'Value', 1, ...
        'Units', 'normalized', 'Position', [0.1 0.08 0.8 0.03], ...
        'SliderStep', [1/num_frames, 10/num_frames]);
    
    addlistener(slider, 'ContinuousValueChange', @slider_callback);
    set(slider, 'Callback', @slider_callback);
    
    txt_time = uicontrol('Style', 'text', 'String', 'Tempo: 0.00 s', ...
        'Units', 'normalized', 'Position', [0.8 0.12 0.1 0.03], ...
        'BackgroundColor', 'w', 'HorizontalAlignment', 'right', 'FontSize', 10);

    btn_prev = uicontrol('Style', 'pushbutton', 'String', '< Step', ...
        'Units', 'normalized', 'Position', [0.35 0.02 0.08 0.04], ...
        'Callback', @prev_callback);
        
    btn_play = uicontrol('Style', 'pushbutton', 'String', 'Play', ...
        'Units', 'normalized', 'Position', [0.45 0.01 0.1 0.06], ...
        'FontWeight', 'bold', 'FontSize', 11, 'Callback', @play_callback);
        
    btn_next = uicontrol('Style', 'pushbutton', 'String', 'Step >', ...
        'Units', 'normalized', 'Position', [0.57 0.02 0.08 0.04], ...
        'Callback', @next_callback);

    update_graphics(current_idx);

    %% 8. Funzioni di Callback (Nested Functions)
    function update_graphics(idx)
        % --- Update Animazione ---
        xw = xw_all(idx);
        yw = yw_all(idx);
        xk = xk_all(idx);
        yk = yk_all(idx);
        xh = xh_all(idx);
        yh = yh_all(idx);
        x_eq = x_eq_all(idx);
        y_eq = y_eq_all(idx);
        qw = qw_int(idx);
        
        set(h_wheel_center, 'XData', xw, 'YData', yw);
        set(h_wheel_circle, 'XData', x_circle_base + xw, 'YData', y_circle_base + yw);
        set(h_wheel_line, 'XData', [xw, xw + Rw*sin(qw)], 'YData', [yw, yw + Rw*cos(qw)]);
        
        set(h_link1, 'XData', [xw xk], 'YData', [yw yk]);
        set(h_link2, 'XData', [xk xh], 'YData', [yk yh]);
        set(h_knee, 'XData', xk, 'YData', yk);
        set(h_hip, 'XData', xh, 'YData', yh);
        set(h_pend_eq_line, 'XData', [xw x_eq], 'YData', [yw y_eq]);
        set(h_pend_eq_mass, 'XData', x_eq, 'YData', y_eq);

        window_width = 2.0;
        xlim(ax_anim, [xw - window_width/2, xw + window_width/2]);
        
        % --- Update Plot Dinamici ---
        current_time = t_anim(idx);
        
        set(h_dot_q1,  'XData', current_time, 'YData', q1_int(idx));
        set(h_dot_q2,  'XData', current_time, 'YData', q2_int(idx));
        set(h_dot_qw,  'XData', current_time, 'YData', qw_int(idx));
        set(h_dot_eq,  'XData', current_time, 'YData', teq_int(idx));
        set(h_dot_tau, 'XData', current_time, 'YData', tau_w_int(idx));
        
        set(h_tbar_q,   'XData', [current_time current_time]);
        set(h_tbar_qw,  'XData', [current_time current_time]);
        set(h_tbar_eq,  'XData', [current_time current_time]);
        set(h_tbar_tau, 'XData', [current_time current_time]);

        set(slider, 'Value', idx);
        set(txt_time, 'String', sprintf('Tempo: %.2f s', current_time));
        
        drawnow;
    end

    function play_callback(~, ~)
        is_playing = ~is_playing;
        
        if is_playing
            set(btn_play, 'String', 'Pausa', 'ForegroundColor', [0.8 0 0]);
            if current_idx >= num_frames
                current_idx = 1;
            end
            
            while is_playing && current_idx < num_frames
                if ~isgraphics(fig), return; end
                
                tic;
                current_idx = current_idx + 1;
                update_graphics(current_idx);
                
                t_elapsed = toc;
                if t_elapsed < dt_anim
                    pause(dt_anim - t_elapsed);
                else
                    pause(0.001); 
                end
            end
            
            is_playing = false;
            if isgraphics(fig)
                set(btn_play, 'String', 'Play', 'ForegroundColor', [0 0 0]);
            end
        else
            set(btn_play, 'String', 'Play', 'ForegroundColor', [0 0 0]);
        end
    end

    function prev_callback(~, ~)
        stop_playback();
        current_idx = max(1, current_idx - 1);
        update_graphics(current_idx);
    end

    function next_callback(~, ~)
        stop_playback();
        current_idx = min(num_frames, current_idx + 1);
        update_graphics(current_idx);
    end

    function slider_callback(hObject, ~)
        stop_playback();
        current_idx = round(get(hObject, 'Value'));
        update_graphics(current_idx);
    end

    function stop_playback()
        is_playing = false;
        set(btn_play, 'String', 'Play', 'ForegroundColor', [0 0 0]);
    end
end