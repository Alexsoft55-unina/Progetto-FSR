function [Fz_opt, Delta_s_opt, J] = MPC(x, x_ref, zb, g, mb, dt, mu, L_max, F_min, F_max, S_bar, W_bar, N) 
    % INPUT: 
    % x_ref DEVE essere un vettore colonna [4*N x 1]. Se è una matrice, 
    % devi srotolarla prima di questa riga.

    h_k = x(3);
    dc = [0; 0; 0; -g];     % Disturbo di gravità
    dk = dc * dt;
    
    % --- Ricalcolo Dinamica Locale basata su h_k ---
    Ac = [0 1 0 0; 0 0 0 0; 0 0 0 1; 0 0 0 0];
    Bc = [0 0; g/h_k 0; 0 0; 0 1/mb];
    Ak = eye(4) + Ac * dt;
    Bk = Bc * dt;
    
    % --- Aggiornamento Dinamico dei Vincoli ---
    max_kinematic_ds = sqrt(L_max^2 - zb^2);
    ds_min = max(-mu*h_k, -max_kinematic_ds);
    ds_max = min(mu*h_k, max_kinematic_ds);
    
    U_min = [ds_min; F_min];
    U_max = [ds_max; F_max];
    LB = repmat(U_min, N, 1);
    UB = repmat(U_max, N, 1);
    
    % --- Calcolo Matrici di Predizione per l'orizzonte N ---
    Phi = zeros(4*N, 2*N);
    Psi = zeros(4*N, 4);
    Term_dist = zeros(4*N, 1);
    
    for i = 1:N
        Psi((i-1)*4+1:i*4, :) = Ak^i;
        for j = 1:i
            Phi((i-1)*4+1:i*4, (j-1)*2+1:j*2) = (Ak^(i-j)) * Bk;
            Term_dist((i-1)*4+1:i*4) = Term_dist((i-1)*4+1:i*4) + (Ak^(i-j))*dk;
        end
    end
    
    H = 2 * (Phi' * S_bar * Phi + W_bar);
    f = 2 * Phi' * S_bar * (Psi * x + Term_dist - x_ref);
    
    % --- Definizione Options (Mancava!) ---
    options = optimoptions('quadprog', 'Display', 'off');
    
    % --- Risoluzione QP ---
    [U_seq, fval, exitflag] = quadprog(H, f, [], [], [], [], LB, UB, [], options);
    
    % --- Failsafe: Protezione contro il crash del solver ---
    if exitflag <= 0 || isempty(U_seq)
        % Se fallisce, forza gli input a zero o a un valore di sicurezza
        U_seq = zeros(2*N, 1);
        fval = 0;
    end
    
    % --- Calcolo del vero costo ---
    Errore_non_controllato = (Psi * x + Term_dist - x_ref);
    cost_costante = Errore_non_controllato' * S_bar * Errore_non_controllato;
    J = fval + cost_costante;
    
    % --- Assegnazione Output Corretta ---
    u_opt = U_seq(1:2);
    Delta_s_opt = u_opt(1); % u_1 controlla la dinamica x
    Fz_opt = u_opt(2);      % u_2 controlla la dinamica z
end