function [q_gomito_basso, q_gomito_alto] = cinematica_inversa_2R(x, y, params)
    % CINEMATICA_INVERSA_2R Calcola gli angoli dei giunti per raggiungere un target cartesiano.
    %
    % Input:
    %   x, y   - Coordinate cartesiane dell'end-effector (m)
    %   params - Struct contenente le lunghezze dei bracci l1 e l2 (m)
    %
    % Output:
    %   q_gomito_basso - Vettore colonna [theta1; theta2] per la prima soluzione (rad)
    %   q_gomito_alto  - Vettore colonna [theta1; theta2] per la seconda soluzione (rad)

    % Estrazione parametri
    l1 = params.l1;
    l2 = params.l2;

    % Calcolo del quadrato della distanza dal target all'origine
    dist_quad = x^2 + y^2;

    % Controllo di raggiungibilità (workspace del robot)
    if dist_quad > (l1 + l2)^2 || dist_quad < (l1 - l2)^2
        error('Errore: Il target (%.2f, %.2f) è fuori dal raggio di azione del robot.', x, y);
    end

    % Calcolo del coseno dell'angolo theta2 (Teorema del coseno)
    c2 = (dist_quad - l1^2 - l2^2) / (2 * l1 * l2);

    % ---------------------------------------------------------
    % Soluzione 1: Gomito in basso (seno di theta2 positivo)
    % ---------------------------------------------------------
    s2_a = sqrt(1 - c2^2);
    theta2_a = atan2(s2_a, c2);
    
    % Calcolo di theta1
    theta1_a = atan2(y, x) - atan2(l2 * s2_a, l1 + l2 * c2);
    
    q_gomito_basso = [theta1_a; theta2_a];

    % ---------------------------------------------------------
    % Soluzione 2: Gomito in alto (seno di theta2 negativo)
    % ---------------------------------------------------------
    s2_b = -sqrt(1 - c2^2);
    theta2_b = atan2(s2_b, c2);
    
    % Calcolo di theta1
    theta1_b = atan2(y, x) - atan2(l2 * s2_b, l1 + l2 * c2);
    
    q_gomito_alto = [theta1_b; theta2_b];
end