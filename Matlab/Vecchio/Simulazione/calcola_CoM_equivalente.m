function [l_eq, theta_eq, S_C, Z_C] = calcola_CoM_equivalente(q, robot_params)
    % CALCOLA_COM_EQUIVALENTE Calcola il Centro di Massa (CoM) equivalente
    % dell'Upper Body e i parametri del pendolo inverso (WIP) associato.
    % Riferimento: Equazioni (1), (2) e (3) del paper.
    %
    % Input:
    %   q            - Vettore di stato 4x1 [q_w; q1; q2; q3] (rad)
    %   robot_params - Struct contenente le masse e le lunghezze dei link
    %
    % Output:
    %   l_eq         - Lunghezza del pendolo equivalente (m)
    %   theta_eq     - Angolo di inclinazione del pendolo equivalente (rad)
    %   S_C, Z_C     - Coordinate cartesiane orizzontale/verticale del CoM 
    %                  dell'Upper-Body rispetto all'asse della ruota (m).

    % --- 1. Estrazione variabili ---
    % q(1) è la ruota (q_w), la ignoriamo perché l'origine è il centro ruota
    q1 = q(1);
    q2 = q(2);
    q3 = q(3); % q3 è assunto come l'angolo assoluto del torso dalla verticale

    l1 = robot_params.l1;
    l2 = robot_params.l2;
    l3 = robot_params.l3;
    
    m1 = robot_params.m1;
    m2 = robot_params.m2;
    m3 = robot_params.m3;

    % --- 2. Matrici di Trasformazione Omogenea (D-H) - Eq. (1) ---
    % T1: Dall'asse ruota al Link 1 (Tibia)
    T1 = [cos(q1), -sin(q1), 0;
          sin(q1),  cos(q1), 0;
          0,        0,       1];

    % T2: Dall'asse ruota al Link 2 (Coscia)
    T2 = [cos(q1+q2), -sin(q1+q2), l1*cos(q1);
          sin(q1+q2),  cos(q1+q2), l1*sin(q1);
          0,           0,          1];

    % T3: Dall'asse ruota al Link 3 (Torso)
    % Nota: dato che q3 è misurato rispetto alla verticale, la matrice R 
    % ha seni e coseni "scambiati" rispetto allo standard
    T3 = [sin(q3),  cos(q3), l1*cos(q1) + l2*cos(q1+q2);
          cos(q3), -sin(q3), l1*sin(q1) + l2*sin(q1+q2);
          0,        0,       1];

    % --- 3. Posizione dei baricentri LOCALI ---
    % Vettori omogenei 3x1 [x_locale; y_locale; 1]. 
    % Assumiamo i baricentri esattamente a metà del braccio.
    P_local_1 = [l1/2; 0; 1];
    P_local_2 = [l2/2; 0; 1];
    P_local_3 = [l3/2; 0; 1];

    % Proiezione nel sistema di riferimento globale (asse ruota)
    P_global_1 = T1 * P_local_1;
    P_global_2 = T2 * P_local_2;
    P_global_3 = T3 * P_local_3;

    % --- 4. Calcolo del CoM Equivalente dell'Upper Body - Eq. (2) ---
    m_upper = m1 + m2 + m3; % Escludiamo m_w!
    
    % Media pesata delle coordinate
    P_C_homo = (m1 * P_global_1 + m2 * P_global_2 + m3 * P_global_3) / m_upper;

    % Estrazione coordinate dal vettore omogeneo
    S_C = P_C_homo(1); % Orizzontale
    Z_C = P_C_homo(2); % Verticale

    % --- 5. Parametri del Pendolo Equivalente - Eq. (3) ---
    l_eq = sqrt(S_C^2 + Z_C^2);
    
    % Usiamo atan2 al posto di atan per gestire automaticamente i quadranti 
    % e non avere problemi di divisione per zero se Z_C diventa piccolissimo
    theta_eq = atan2(S_C, Z_C); 
end