% Pulizia iniziale
clear; clc;

% Variabili fisiche e geometriche costanti
syms Rw mw m1 m2 m3 l1 l2 l3 g Iw I1 I2 I3 real

% 1. IL TRUCCO: Variabili statiche indipendenti (niente (t)!)
syms qw q1 q2 q3 real           % Posizioni
syms dqw dq1 dq2 dq3 real       % Velocità
syms ddqw ddq1 ddq2 ddq3 real   % Accelerazioni

Q   = [qw; q1; q2; q3];
dQ  = [dqw; dq1; dq2; dq3];
ddQ = [ddqw; ddq1; ddq2; ddq3];

%% Modello cinematico
sw = Rw*qw;

% Posizioni dei baricentri
x1 = l1/2*cos(q1) + sw;
y1 = l1/2*sin(q1);

x2 = l1*cos(q1) + l2/2*cos(q1 + q2) + sw;
y2 = l1*sin(q1) + l2/2*sin(q1 + q2);

x3 = l1*cos(q1) + l2*cos(q1 + q2) + l3/2*sin(q3) + sw;
y3 = l1*sin(q1) + l2*sin(q1 + q2) + l3/2*cos(q3);

%% Calcolo delle energie cinetiche
% Calcolo rigoroso delle velocità lineari (Regola della catena v = J*dq)
v1_x = jacobian(x1, Q) * dQ;
v1_y = jacobian(y1, Q) * dQ;

v2_x = jacobian(x2, Q) * dQ;
v2_y = jacobian(y2, Q) * dQ;

v3_x = jacobian(x3, Q) * dQ;
v3_y = jacobian(y3, Q) * dQ;

% Velocità angolari
omega_w = dqw;
omega_1 = dq1;
omega_2 = dq1 + dq2;
omega_3 = dq3;

% Energie Cinetiche
Tw = 0.5 * mw * (Rw*dqw)^2 + 0.5 * Iw * omega_w^2;
T1 = 0.5 * m1 * (v1_x^2 + v1_y^2) + 0.5 * I1 * omega_1^2;
T2 = 0.5 * m2 * (v2_x^2 + v2_y^2) + 0.5 * I2 * omega_2^2;
T3 = 0.5 * m3 * (v3_x^2 + v3_y^2) + 0.5 * I3 * omega_3^2;

T = simplify(Tw + T1 + T2 + T3);

%% Calcolo delle energie potenziali
Uw = mw * g * Rw;
U1 = m1 * g * (Rw + y1);
U2 = m2 * g * (Rw + y2);
U3 = m3 * g * (Rw + y3);

U = simplify(Uw + U1 + U2 + U3);

%% Calcolo Lagrangiana
L = T - U;

%% Equazione di Eulero-Lagrange (Metodo Algebrico Sicuro)
disp('Calcolo delle equazioni di Eulero-Lagrange in corso...');

% 1. Derivata parziale rispetto alle posizioni (dL/dq)
dL_dq = jacobian(L, Q)';

% 2. Derivata parziale rispetto alle velocità (dL/dq_dot)
dL_ddq = jacobian(L, dQ)';

% 3. Derivata temporale totale di dL/dq_dot (Regola della Catena Esplicita)
d_dt_dL_ddq = jacobian(dL_ddq, Q) * dQ + jacobian(dL_ddq, dQ) * ddQ;

% 4. Equazioni di moto
Eq_moto = simplify(d_dt_dL_ddq - dL_dq);

disp('Calcolo completato!');

%% Estrazione Matrici Dinamiche (M, C, G)
disp('--------------------------------------------------');
disp('VETTORE DI GRAVITA G(q):');
% G(q) si ottiene annullando velocità e accelerazioni (sostituisco con zeri)
G_matrix = simplify(subs(Eq_moto, [dQ; ddQ], zeros(8,1)));
disp(G_matrix);

disp('--------------------------------------------------');
disp('MATRICE DI MASSA/INERZIA M(q):');
% M(q) è semplicemente lo Jacobiano delle equazioni rispetto alle accelerazioni
M_matrix = simplify(jacobian(Eq_moto, ddQ));
disp(M_matrix);

disp('--------------------------------------------------');
disp('VETTORE DELLE FORZE DI CORIOLIS V(q, dq):');
% V = Eq - M*ddq - G
V_vector = simplify(Eq_moto - M_matrix*ddQ - G_matrix);
disp(V_vector);

disp('--------------------------------------------------');
disp('MATRICE C(q, dq) RIGOROSA (Simboli di Christoffel):');
n = 4;
C_matrix = sym(zeros(n, n));

for i = 1:n
    for j = 1:n
        for k = 1:n
            % Simbolo di Christoffel (usiamo le variabili statiche Q)
            c_ijk = 0.5 * (diff(M_matrix(i,j), Q(k)) + ...
                           diff(M_matrix(i,k), Q(j)) - ...
                           diff(M_matrix(j,k), Q(i)));
            
            C_matrix(i,j) = C_matrix(i,j) + c_ijk * dQ(k);
        end
    end
end

C_matrix = simplify(C_matrix);
disp(C_matrix);

% Verifica Matematica Finale
verifica = simplify(C_matrix * dQ - V_vector);
disp('Verifica C * dq == V (Se restituisce 0, il calcolo è perfetto!):');
disp(verifica);
disp('--------------------------------------------------');