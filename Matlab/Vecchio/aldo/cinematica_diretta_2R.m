function [x, y] = cinematica_diretta_2R(q, params)
    % CINEMATICA_DIRETTA_2R Calcola la posizione (x,y) dell'end-effector.
    % 
    % Input:
    %   q      - Vettore colonna 2x1 [theta1; theta2] (rad)
    %   params - Struct con l1 e l2 (m)
    %
    % Output:
    %   x, y   - Coordinate cartesiane dell'end-effector (m)

    theta1 = q(1);
    theta2 = q(2);
    
    l1 = params.l1;
    l2 = params.l2;

    % Equazioni cinematiche del robot planare 2R
    x = l1 * cos(theta1) + l2 * cos(theta1 + theta2);
    y = l1 * sin(theta1) + l2 * sin(theta1 + theta2);
end