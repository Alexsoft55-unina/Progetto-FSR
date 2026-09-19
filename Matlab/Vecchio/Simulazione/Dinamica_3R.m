clear; clc;

disp('1. Inizializzazione variabili simboliche...');
syms q1 q2 q3 dq1 dq2 dq3 real
syms m1 m2 m3 l1 l2 l3 lc1 lc2 lc3 I1 I2 I3 g real

q = [q1; q2; q3];
dq = [dq1; dq2; dq3];

disp('2. Calcolo Cinematica Diretta dei baricentri...');
% Posizioni dei baricentri dei 3 link rispetto alla base (0,0)
x1 = lc1*cos(q1);
y1 = lc1*sin(q1);

x2 = l1*cos(q1) + lc2*cos(q1+q2);
y2 = l1*sin(q1) + lc2*sin(q1+q2);

x3 = l1*cos(q1) + l2*cos(q1+q2) + lc3*cos(q1+q2+q3);
y3 = l1*sin(q1) + l2*sin(q1+q2) + lc3*sin(q1+q2+q3);

% Velocità dei baricentri (Jacobiani traslazionali)
v1 = [jacobian(x1, q)*dq; jacobian(y1, q)*dq];
v2 = [jacobian(x2, q)*dq; jacobian(y2, q)*dq];
v3 = [jacobian(x3, q)*dq; jacobian(y3, q)*dq];

disp('3. Calcolo Energia Cinetica (T) e Potenziale (U)...');
% Velocità angolari assolute
omega1 = dq1;
omega2 = dq1 + dq2;
omega3 = dq1 + dq2 + dq3;

% Energia Cinetica (Traslazionale + Rotazionale)
T1 = 0.5*m1*(v1'*v1) + 0.5*I1*omega1^2;
T2 = 0.5*m2*(v2'*v2) + 0.5*I2*omega2^2;
T3 = 0.5*m3*(v3'*v3) + 0.5*I3*omega3^2;
T = simplify(T1 + T2 + T3);

% Energia Potenziale (Gravità lungo l'asse Y)
U = m1*g*y1 + m2*g*y2 + m3*g*y3;

disp('4. Estrazione Matrici M(q), C(q,dq), G(q)...');
% Vettore di Gravità G(q)
G_matrix = simplify(jacobian(U, q)');

% Matrice di Inerzia M(q)
% Si ricava derivando due volte l'energia cinetica rispetto alle velocità
M_matrix = simplify(hessian(T, dq));

% Matrice di Coriolis C(q,dq) usando i Simboli di Christoffel
n = 3;
C_matrix = sym(zeros(n,n));
for i = 1:n
    for j = 1:n
        for k = 1:n
            c_ijk = 0.5 * (diff(M_matrix(i,j), q(k)) + ...
                           diff(M_matrix(i,k), q(j)) - ...
                           diff(M_matrix(j,k), q(i)));
            C_matrix(i,j) = C_matrix(i,j) + c_ijk * dq(k);
        end
    end
end
C_matrix = simplify(C_matrix);

disp('5. Generazione della funzione MATLAB numerica...');
% Raggruppiamo i parametri per comodità
params = [m1, m2, m3, l1, l2, l3, lc1, lc2, lc3, I1, I2, I3, g];

% Crea una funzione autonoma su file
matlabFunction(M_matrix, C_matrix, G_matrix, ...
    'File', 'dinamica_3R_numerica', ...
    'Vars', {q, dq, params}, ...
    'Outputs', {'M', 'C', 'G'}, ...
    'Optimize', true);

disp('COMPLETATO! Il file "dinamica_3R_numerica.m" è stato creato.');

%% ========================================================================
%% ESEMPIO DI UTILIZZO CON VALORI NUMERICI
%% ========================================================================
disp('--- TEST DELLA FUNZIONE GENERATA ---');

% 1. Definiamo un set di parametri fisici (m, l, lc, I, g)
p_num = [1.5, 1.0, 0.8, ...       % Masse (m1, m2, m3)
         0.5, 0.4, 0.3, ...       % Lunghezze link (l1, l2, l3)
         0.25, 0.2, 0.15, ...     % Baricentri (lc1, lc2, lc3)
         0.03, 0.02, 0.01, ...    % Inerzie (I1, I2, I3)
         9.81];                   % Gravità (g)

% 2. Definiamo lo stato attuale dei 3 giunti
q_attuale  = [pi/4; -pi/6; pi/3];   % Posizioni [rad]
dq_attuale = [0.1; 0.5; -0.2];      % Velocità [rad/s]

% 3. Richiamiamo la funzione appena creata!
[M_num, C_num, G_num] = dinamica_3R_numerica(q_attuale, dq_attuale, p_num);

disp('Matrice di Inerzia M(q):'); disp(M_num);
disp('Matrice di Coriolis C(q,dq):'); disp(C_num);
disp('Vettore di Gravità G(q):'); disp(G_num);