    % =========================================================================
    % Script di Inizializzazione — Modello Lagrangiano Robot Bipede "Scooter"
    % Convenzione ESATTA del paper (Cui et al., 2022, Table 1):
    %   qw = rotazione ruota
    %   q1 = angolo GINOCCHIO (attuato)
    %   q2 = angolo ANCA (attuato)
    %   q3 = angolo TORSO, assoluto rispetto alla verticale (NON attuato,
    %        evolve per dinamica libera)
    %   q0 = angolo CAVIGLIA (NON attuato, vincolo algebrico: q0 = q1 - q2 + q3)
    %
    % q1=q2=q3=0  ->  robot perfettamente dritto in piedi, tutti i link verticali.
    % =========================================================================
    clc; clear;
   
    run("robot_parameters.m");

    % ---- Riferimenti di postura -------------------------------------------
    K.q1_ref  = -pi/3;          % GINOCCHIO  [rad]
    K.q2_ref  =  pi/6; pi/6;         % ANCA       [rad]   (q3 NON ha riferimento: e' libero)
    
    % ---- Anello esterno: v_com -> theta_b* (fase non minima, LENTO) --------
    K.KP      = -0.5;-1;0.3;
    K.KI      = -0.5;-6;
    K.TH_MAX  = deg2rad(12);
    
    % ---- Canale 1: theta_b, super-twisting ---------------------------------
    K.c1      = -1e2;-100;
    K.ka      = 10;
    K.kb      = 50;
    K.eps1    = 0.01;          % regolarizzazione del sign (anti-chattering)
    
    % ---- Canali 2 e 3: q1, q2 — SMC classico con boundary layer ------------
    K.c2 = 1e4/100;   K.k2 = 1.0;   K.eta2 = 1.0*10;   K.phi2 = 0.075;   % GINOCCHIO
    K.c3 = 1e4/100;  K.k3 = 1.0;   K.eta3 = 1.0*10;   K.phi3 =0.075;   % ANCA
    K.ki2 = 0*80;
    K.ki3 = 80;
    % ---- Limiti attuatori e aderenza ---------------------------------------
    K.TAU_MAX = [500; 100; 100];        % [tau_w ; tau_1(gin.) ; tau_2(anca)]
    K.mu      = 0.8;
    K.N_nom   = (robot_params.mw+robot_params.m1+robot_params.m2+robot_params.m3/2)*robot_params.g;
    K.Rw      = robot_params.Rw;
    
   
    %B = [-1 1 0; 1 0 0; 1 0 1; -1 0 0];
    B = [0 1 0; 1 0 0; 0 0 1; 0 0 0];
    
    %% Initial Joint Position
    q_init =zeros(6,1);
    q0_guess = [pi/3; -pi/6; pi/3];
    dq_init = [0;0;0;0];
    %q_init = [0.52,-1.04, 0,0.52,-1.04,0];
    %q_init = [pi/6,-pi/3, 0,pi/6,-pi/3,0];

    %% Sample time actuator
    
    Ts=1e-4;
    
    
    
    %% Motor parameters
    
    Motor_damping = 0.25;
    Motor_stiffness = 0.0;
    

    % %% 2. Derivazione simbolica Lagrangiana
    % syms Rw mw m1 m2 m3 l1 l2 l3 gs Iw I1 I2 I3 real
    % syms qw q1 q2 q3 real
    % syms dqw dq1 dq2 dq3 real
    % syms ddqw ddq1 ddq2 ddq3 real
    % 
    % Q   = [q1; qw; q2; q3];
    % dQ  = [dq1; dqw; dq2; dq3];
    % ddQ = [ddq1; ddqw; ddq2; ddq3];
    % 
    % % --- Cinematica (angoli assoluti dalla verticale, convenzione paper) ---
    % sw = Rw*qw;
    % 
    % % Angolo assoluto stinco: q0 = q1 - q2 + q3  (caviglia, vincolo algebrico)
    % phi_shank = q1 - q2 + q3;
    % 
    % % Angolo assoluto coscia: q3 - q2
    % phi_thigh = q3 - q2;
    % 
    % % Angolo assoluto torso: q3
    % 
    % % Stinco (link1)
    % x1 = sw + (l1/2)*sin(phi_shank);
    % y1 = Rw + (l1/2)*cos(phi_shank);
    % 
    % % Estremo superiore stinco = ginocchio
    % xk = sw + l1*sin(phi_shank);
    % yk = Rw + l1*cos(phi_shank);
    % 
    % % Coscia (link2)
    % x2 = xk + (l2/2)*sin(phi_thigh);
    % y2 = yk + (l2/2)*cos(phi_thigh);
    % 
    % % Estremo superiore coscia = anca
    % xh = xk + l2*sin(phi_thigh);
    % yh = yk + l2*cos(phi_thigh);
    % 
    % % Torso (link3)
    % x3 = xh + (l3/2)*sin(q3);
    % y3 = yh + (l3/2)*cos(q3);
    % 
    % v1 = [jacobian(x1, Q)*dQ; jacobian(y1, Q)*dQ];
    % v2 = [jacobian(x2, Q)*dQ; jacobian(y2, Q)*dQ];
    % v3 = [jacobian(x3, Q)*dQ; jacobian(y3, Q)*dQ];
    % 
    % omega_w = dqw;
    % omega_shank = dq1 - dq2 + dq3;   % velocità angolare assoluta stinco
    % omega_thigh = dq3 - dq2;         % velocità angolare assoluta coscia
    % omega_torso = dq3;               % velocità angolare assoluta torso
    % 
    % % --- Energie ---
    % Tw = 0.5*mw*(Rw*dqw)^2 + 0.5*Iw*omega_w^2;
    % T1 = 0.5*m1*(v1'*v1) + 0.5*I1*omega_shank^2;
    % T2 = 0.5*m2*(v2'*v2) + 0.5*I2*omega_thigh^2;
    % T3 = 0.5*m3*(v3'*v3) + 0.5*I3*omega_torso^2;
    % T = simplify(Tw + T1 + T2 + T3);
    % 
    % U = simplify(mw*gs*Rw + m1*gs*y1 + m2*gs*y2 + m3*gs*y3);
    % 
    % % --- Equazioni di Eulero-Lagrange ---
    % L = T - U;
    % Eq_moto = simplify( jacobian(jacobian(L, dQ)', Q)*dQ ...
    %                    + jacobian(jacobian(L, dQ)', dQ)*ddQ ...
    %                    - jacobian(L, Q)' );
    % 
    % G_matrix = simplify(subs(Eq_moto, [dQ; ddQ], zeros(8,1)));
    % M_matrix = simplify(jacobian(Eq_moto, ddQ));
    % C_matrix_times_dQ = simplify(Eq_moto - M_matrix*ddQ - G_matrix);
    % 
    % %% 3. Verifica statica dell'equilibrio PRIMA di generare il file
    % params_sym = [Rw, mw, m1, m2, m3, l1, l2, l3, gs, Iw, I1, I2, I3];
    % params_num = [robot_params.Rw, robot_params.mw, robot_params.m1, ...
    %               robot_params.m2, robot_params.m3, robot_params.l1, ...
    %               robot_params.l2, robot_params.l3, robot_params.g, ...
    %               robot_params.Iw, robot_params.I1, robot_params.I2, robot_params.I3];
    % 
    % G_check = double(subs(G_matrix, [params_sym, Q.'], [params_num, 0,0,0,0]));
    % disp('Verifica G(q=0) [atteso: componenti su q1,q2,q3 ≈ 0, robot verticale in equilibrio]:');
    % disp(G_check');
    % 
    % % Verifica aggiuntiva: q0 (caviglia) a q1=q2=q3=0 deve dare 0
    % q0_check = 0 - 0 + 0;
    % fprintf('Verifica q0 (caviglia) a q=0: %.4f (atteso: 0)\n', q0_check);
    % %% 5. Jacobiano del baricentro (CoM) rispetto ai giunti attuati (q1, q2)
    % disp('Derivazione Jacobiano CoM per il VMC...');
    % 
    % % Versione "relativa all'asse ruota" (senza sw, così non dipende da qw)
    % x1_rel = (l1/2)*sin(phi_shank);
    % y1_rel = (l1/2)*cos(phi_shank);
    % 
    % xk_rel = l1*sin(phi_shank);
    % yk_rel = l1*cos(phi_shank);
    % 
    % x2_rel = xk_rel + (l2/2)*sin(phi_thigh);
    % y2_rel = yk_rel + (l2/2)*cos(phi_thigh);
    % 
    % xh_rel = xk_rel + l2*sin(phi_thigh);
    % yh_rel = yk_rel + l2*cos(phi_thigh);
    % 
    % x3_rel = xh_rel + (l3/2)*sin(q3);
    % y3_rel = yh_rel + (l3/2)*cos(q3);
    % 
    % SC_sym = (m1*x1_rel + m2*x2_rel + m3*x3_rel) / (m1+m2+m3);
    % ZC_sym = (m1*y1_rel + m2*y2_rel + m3*y3_rel) / (m1+m2+m3);
    % 
    % J_SC = [diff(SC_sym, q1), diff(SC_sym, q2)];
    % J_ZC = [diff(ZC_sym, q1), diff(ZC_sym, q2)];
    % 
    % matlabFunction(subs(J_SC, params_sym, params_num), ...
    %                subs(J_ZC, params_sym, params_num), ...
    %                subs(SC_sym, params_sym, params_num), ...
    %                subs(ZC_sym, params_sym, params_num), ...
    %                'File', 'calc_com_jacobian', ...
    %                'Vars', {q1, q2, q3}, ...
    %                'Outputs', {'J_SC', 'J_ZC', 'SC', 'ZC'});
    % 
    % disp('File calc_com_jacobian.m generato con successo!');
    % %% 4. Generazione calc_dynamics.m
    % disp('Generazione del file calc_dynamics.m in corso...');
    % matlabFunction(subs(M_matrix, params_sym, params_num), ...
    %                subs(C_matrix_times_dQ, params_sym, params_num), ...
    %                subs(G_matrix, params_sym, params_num), ...
    %                'File', 'calc_dynamics', ...
    %                'Vars', {Q, dQ}, ...
    %                'Outputs', {'M', 'CdQ', 'G'});
    % disp('File calc_dynamics.m generato con successo!');
    % 
    % disp('=========================================================');
    % disp('Inizializzazione completa.');
    % disp('Convenzione: q1=ginocchio, q2=anca, q3=torso(assoluto), qw=ruota.');
    % disp('q0 (caviglia, PASSIVA) = q1 - q2 + q3, calcolata solo per plot/monitoraggio.');
    % disp('=========================================================');