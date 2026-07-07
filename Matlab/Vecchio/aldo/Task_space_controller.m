
function [tau_hip,tau_knee]= Task_space_controller(Fz_cmd, delta_s, h_cmd,l1,l2)
%% Stance Task-Space Controller (Calcolo Coppie Giunti - Eq 19)

    % --- 1. Cinematica Inversa (IK) ---
    L_sq = delta_s^2 + h_cmd^2;
    L = sqrt(L_sq);
    
    alpha = atan2(h_cmd, delta_s); 
    cos_beta = (l1^2 + L_sq - l2^2) / (2 * l1 * L);
    cos_beta = max(min(cos_beta, 1), -1); 
    beta = acos(cos_beta);
    
    q1 = alpha - beta; 
    xk = l1 * cos(q1);
    yk = l1 * sin(q1);
    q2 = atan2(h_cmd - yk, delta_s - xk);
  
    % --- 2. Calcolo dello Jacobiano (J) ---
    J11 = -l1 * sin(q1);
    J12 = -l2 * sin(q2);
    J21 =  l1 * cos(q1);
    J22 =  l2 * cos(q2);
    J = [J11, J12; J21, J22];
         
    % --- 3. VIRTUAL MODEL CONTROL (EQUAZIONE 19 COMPLETA) ---
    % Vettori Desiderati (d)
    p_d = [-delta_s; -h_cmd]; % Posizione desiderata ruota rispetto all'anca
    v_d = [0; 0];             % Velocità relativa desiderata
    
    % Vettori Effettivi (f) - (In simulazione ideale coincidono con i desiderati)
    p_f = [-xk; -yk];         
    v_f = [0; 0];             
    
    % Calcolo Feedback (Molla e Smorzatore)
    F_feedback = k_p * (p_d - p_f) + k_d * (v_d - v_f);
    
    % Calcolo Feedforward (Forze ottimali dall'MPC)
    F_feedforward = [0; Fz_cmd]; 
    tau_ff = J' * F_feedforward;
    
    % EQUAZIONE 19
    Tau_abs = J' * F_feedback + tau_ff; 
    
    % --- 4. Coppie Relative dei Giunti ---
    tau_hip = Tau_abs(2); 
    tau_knee = Tau_abs(1); %Tau_abs(1) - Tau_abs(2); 

