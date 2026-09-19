function animate_robot(out)
    % ANIMATE_ROBOT_UI Riproduzione grafica interattiva della dinamica.
    % Include controlli UI per Play/Pausa, Step e Slider temporale.
    
    %% 1. Estrazione e Parsing dei Dati da Simulink
    try
        t = out.tout;
        
        if isprop(out, 'q') && isa(out.q, 'timeseries')
            q_data = out.q.Data;
        elseif isprop(out, 'q')
            q_data = out.q; 
        else
            error('Variabile "q" non trovata.');
        end
        
        if isprop(out, 'pend_eq') && isa(out.pend_eq, 'timeseries')
            pend_eq_data = out.pend_eq.Data;
        elseif isprop(out, 'pend_eq')
            pend_eq_data = out.pend_eq;
        else
            error('Variabile "pend_eq" non trovata.');
        end
        
        if size(q_data, 2) > size(q_data, 1), q_data = q_data'; end
        if size(pend_eq_data, 2) > size(pend_eq_data, 1), pend_eq_data = pend_eq_data'; end
        
    catch ME
        fprintf('Errore nell''estrazione dei dati.\n');
        rethrow(ME);
    end

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
    t_anim = t(1):dt_anim:t(end);
    num_frames = length(t_anim);

    q1_int = interp1(t, q1, t_anim);
    qw_int = interp1(t, qw, t_anim);
    q2_int = interp1(t, q2, t_anim);
    leq_int = interp1(t, l_eq, t_anim);
    teq_int = interp1(t, theta_eq, t_anim);

    %% 4. Pre-calcolo Vettorizzato della Cinematica (Per UI fluida)
    % Calcoliamo tutte le posizioni in anticipo per massimizzare le
    % performance quando si sposta brutalmente lo slider.
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
    fig = figure('Name', 'Animazione UI - Doppio Pendolo', ...
                 'Color', 'w', 'Position', [100 100 900 700]);
             
    % Lasciamo spazio in basso per la UI (Position: [left bottom width height])
    ax = axes(fig, 'Position', [0.1 0.25 0.8 0.7]);
    hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
    
    y_lim = [-0.1, l1 + l2 + 0.4];
    ylim(ax, y_lim);
    xlabel(ax, 'Posizione X [m]', 'FontWeight', 'bold');
    ylabel(ax, 'Posizione Y [m]', 'FontWeight', 'bold');
    title(ax, 'Dinamica del Robot e Modello Equivalente', 'FontSize', 12);

    %% 6. Creazione degli Oggetti Grafici
    c_wheel = [0.2 0.2 0.2];
    c_link1 = [0.0 0.45 0.74];  
    c_link2 = [0.85 0.33 0.10]; 
    c_eq    = [0.47 0.67 0.19]; 
    
    plot(ax, [-100 100], [0 0], 'k', 'LineWidth', 2); % Terreno

    h_wheel_center = plot(ax, 0, Rw, 'o', 'MarkerSize', 6, 'MarkerFaceColor', c_wheel, 'MarkerEdgeColor', 'k');
    h_wheel_circle = plot(ax, 0, 0, 'Color', c_wheel, 'LineWidth', 2); 
    h_wheel_line   = plot(ax, [0 0], [0 Rw], 'Color', c_wheel, 'LineWidth', 2);
    
    h_link1 = plot(ax, [0 0], [0 0], 'Color', c_link1, 'LineWidth', 4);
    h_link2 = plot(ax, [0 0], [0 0], 'Color', c_link2, 'LineWidth', 4);
    
    h_knee = plot(ax, 0, 0, 'ko', 'MarkerFaceColor', c_link1, 'MarkerSize', 8);
    h_hip  = plot(ax, 0, 0, 'ko', 'MarkerFaceColor', c_link2, 'MarkerSize', 10);

    h_pend_eq_line = plot(ax, [0 0], [0 0], '--', 'Color', c_eq, 'LineWidth', 2.5);
    h_pend_eq_mass = plot(ax, 0, 0, 's', 'MarkerFaceColor', c_eq, 'MarkerEdgeColor', 'k', 'MarkerSize', 12);

    legend(ax, [h_link1, h_link2, h_pend_eq_line], ...
           {'Stinco (Link 1)', 'Coscia (Link 2)', 'Pendolo Equivalente'}, ...
           'Location', 'northeast');

    %% 7. Creazione Componenti UI (Pulsanti e Slider)
    % Stato globale dell'animazione
    current_idx = 1;
    is_playing = false;

    % Slider temporale
    slider = uicontrol('Style', 'slider', ...
        'Min', 1, 'Max', num_frames, 'Value', 1, ...
        'Units', 'normalized', 'Position', [0.1 0.12 0.8 0.03], ...
        'SliderStep', [1/num_frames, 10/num_frames], ...
        'Callback', @slider_callback);
    
    % Testo del tempo
    txt_time = uicontrol('Style', 'text', 'String', 'Tempo: 0.00 s', ...
        'Units', 'normalized', 'Position', [0.8 0.16 0.1 0.03], ...
        'BackgroundColor', 'w', 'HorizontalAlignment', 'right');

    % Pulsanti
    btn_prev = uicontrol('Style', 'pushbutton', 'String', '< Step', ...
        'Units', 'normalized', 'Position', [0.35 0.04 0.08 0.05], ...
        'Callback', @prev_callback);
        
    btn_play = uicontrol('Style', 'pushbutton', 'String', 'Play', ...
        'Units', 'normalized', 'Position', [0.45 0.03 0.1 0.07], ...
        'FontWeight', 'bold', 'Callback', @play_callback);
        
    btn_next = uicontrol('Style', 'pushbutton', 'String', 'Step >', ...
        'Units', 'normalized', 'Position', [0.57 0.04 0.08 0.05], ...
        'Callback', @next_callback);

    % Inizializza primo frame
    update_graphics(current_idx);

    %% 8. Funzioni di Callback e Gestione Logica (Nested Functions)

    function update_graphics(idx)
        % Aggiorna le posizioni grafiche in base all'indice corrente
        xw = xw_all(idx);
        yw = yw_all(idx);
        xk = xk_all(idx);
        yk = yk_all(idx);
        xh = xh_all(idx);
        yh = yh_all(idx);
        x_eq = x_eq_all(idx);
        y_eq = y_eq_all(idx);
        qw = qw_int(idx);

        % Update Ruota
        set(h_wheel_center, 'XData', xw, 'YData', yw);
        set(h_wheel_circle, 'XData', x_circle_base + xw, 'YData', y_circle_base + yw);
        set(h_wheel_line, 'XData', [xw, xw + Rw*sin(qw)], 'YData', [yw, yw + Rw*cos(qw)]);
        
        % Update Robot
        set(h_link1, 'XData', [xw xk], 'YData', [yw yk]);
        set(h_link2, 'XData', [xk xh], 'YData', [yk yh]);
        set(h_knee, 'XData', xk, 'YData', yk);
        set(h_hip, 'XData', xh, 'YData', yh);

        % Update Equivalente
        set(h_pend_eq_line, 'XData', [xw x_eq], 'YData', [yw y_eq]);
        set(h_pend_eq_mass, 'XData', x_eq, 'YData', y_eq);

        % Camera Tracking
        window_width = 2.0;
        xlim(ax, [xw - window_width/2, xw + window_width/2]);
        
        % Aggiornamento UI
        set(slider, 'Value', idx);
        set(txt_time, 'String', sprintf('Tempo: %.2f s', t_anim(idx)));

        drawnow;
    end

    function play_callback(~, ~)
        % Alterna stato Play/Pausa
        is_playing = ~is_playing;
        
        if is_playing
            set(btn_play, 'String', 'Pausa', 'ForegroundColor', [0.8 0 0]);
            % Se eravamo alla fine, ripartiamo dall'inizio
            if current_idx >= num_frames
                current_idx = 1;
            end
            
            % Loop di riproduzione
            while is_playing && current_idx < num_frames
                if ~isgraphics(fig)
                    return; % Esce se la figura è stata chiusa
                end
                
                tic;
                current_idx = current_idx + 1;
                update_graphics(current_idx);
                
                % Gestione framerate
                t_elapsed = toc;
                if t_elapsed < dt_anim
                    pause(dt_anim - t_elapsed);
                else
                    pause(0.001); % Permette l'interruzione via pulsanti
                end
            end
            
            % Reset pulsante a fine corsa
            is_playing = false;
            if isgraphics(fig)
                set(btn_play, 'String', 'Play', 'ForegroundColor', [0 0 0]);
            end
        else
            set(btn_play, 'String', 'Play', 'ForegroundColor', [0 0 0]);
        end
    end

    function prev_callback(~, ~)
        % Ferma riproduzione e vai indietro di 1 frame
        stop_playback();
        current_idx = max(1, current_idx - 1);
        update_graphics(current_idx);
    end

    function next_callback(~, ~)
        % Ferma riproduzione e vai avanti di 1 frame
        stop_playback();
        current_idx = min(num_frames, current_idx + 1);
        update_graphics(current_idx);
    end

    function slider_callback(hObject, ~)
        % Ferma riproduzione e salta al frame indicato dallo slider
        stop_playback();
        current_idx = round(get(hObject, 'Value'));
        update_graphics(current_idx);
    end

    function stop_playback()
        % Helper per forzare la pausa durante le iterazioni manuali
        is_playing = false;
        set(btn_play, 'String', 'Play', 'ForegroundColor', [0 0 0]);
    end

end