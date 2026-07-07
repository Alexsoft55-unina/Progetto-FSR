function [tau_hip, tau_knee] = new_Task_space_controller(Fz_cmd, delta_s, h_cmd, k_p, k_d, robot_params)
    %% 1. Cinematica Inversa (IK) - Modello D-H Standard
    % Target: [delta_s; h_cmd] (posizione end-effector)
    % L'ipotenusa L

    l1 = robot_params.l1;
    l2 = robot_params.l2;

    dist_sq = delta_s^2 + h_cmd^2;
    L = sqrt(dist_sq);
    
    % Legge del coseno per q2 (relativo)
    cos_q2 = (dist_sq - l1^2 - l2^2) / (2 * l1 * l2);
    cos_q2 = max(min(cos_q2, 1), -1); % Saturazione numerica
    q2 = acos(cos_q2); % Configurazione: ginocchio flesso positivo
    
    % Angolo q1 (anca)
    q1 = atan2(h_cmd, delta_s) - atan2(l2*sin(q2), l1 + l2*cos(q2));
    
    %% 2. Jacobiano Analitico (D-H Standard)
    % J = [dx/dq1, dx/dq2; dy/dq1, dy/dq2]
    s1 = sin(q1); c1 = cos(q1);
    s12 = sin(q1 + q2); c12 = cos(q1 + q2);
    
    J = [ -l1*s1 - l2*s12, -l2*s12;
           l1*c1 + l2*c12,  l2*c12 ];
         
    %% 3. VIRTUAL MODEL CONTROL
    % Posizione end-effector calcolata dalla cinematica diretta (D-H)
    x_e = l1*c1 + l2*c12;
    y_e = l1*s1 + l2*s12;
    p_f = [x_e; y_e];
    p_d = [delta_s; h_cmd];
    
    % Velocità (assumiamo 0 per semplicità, ma andrebbero lette dai giunti)
    v_f = [0; 0];
    v_d = [0; 0];
    
    % Forza di controllo nel Task Space
    F_feedback = k_p * (p_d - p_f) + k_d * (v_d - v_f);
    F_feedforward = [0; Fz_cmd]; 
    F_tot = F_feedback + F_feedforward;
    
    %% 4. Mapping delle coppie (J' * F)
    % La trasposta dello Jacobiano mappa le forze cartesiane nelle coppie ai giunti
    Tau = J' * F_tot;
    
    % Assegnazione corretta: 
    % Tau(1) -> Giunto 1 (Hip), Tau(2) -> Giunto 2 (Knee)
    tau_hip  = Tau(1); 
    tau_knee = Tau(2); 
end