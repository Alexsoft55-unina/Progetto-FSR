% =========================================================================
% Script di Inizializzazione — Modello Lagrangiano Bipede 2-Link
% Convenzione coordinate generalizzate:
%   Q = [q1; qw; q2]
%   q1  = angolo stinco rispetto alla verticale (rad)
%   qw  = rotazione ruota (rad)
%   q2  = angolo coscia rispetto alla verticale (rad)
%   meq = massa concentrata all'estremo del link 2 (anca/corpo)
% =========================================================================
%% 1. Parametri fisici numerici
run("init.m");

%% 1. Variabili simboliche — TUTTO simbolico, nessun valore numerico nella cinematica
syms Rw mw m1 m2 meq l1 l2 lc1 lc2 g Iw I1 I2 real
syms b1 bw b2 real
syms qw q1 q2 real
syms dqw dq1 dq2 real
syms ddqw ddq1 ddq2 real

% Vettori stato simbolici — ordine: [q1; qw; q2]
Q   = [q1;  qw;  q2];
dQ  = [dq1; dqw; dq2];
ddQ = [ddq1; ddqw; ddq2];

%% 3. Cinematica diretta (forward kinematics) — tutto in termini simbolici

% Spostamento orizzontale del punto di contatto ruota (no slittamento)
sw = Rw * qw;

% --- Centro di massa (CoM) stinco (link 1) ---
% Origine: centro ruota = (sw, Rw)
% Angolo q1 misurato dalla verticale: sin -> componente x, cos -> componente y
x_c1 = sw  + lc1 * sin(q1);
y_c1 = Rw  + lc1 * cos(q1);

% --- Posizione ginocchio (estremo link 1, origine link 2) ---
xk = sw + l1 * sin(q1);
yk = Rw + l1 * cos(q1);

% --- CoM coscia (link 2) ---
x_c2 = xk + lc2 * sin(q2);
y_c2 = yk + lc2 * cos(q2);

% --- Estremo link 2 (anca / massa concentrata meq) ---
xh = xk + l2 * sin(q2);
yh = yk + l2 * cos(q2);

%% 4. Jacobiani traslazionali e velocità punti notevoli

% Jacobiano traslazionale CoM stinco [2x3]
Jc1 = jacobian([x_c1; y_c1], Q);   % righe: x,y — colonne: q1,qw,q2

% Jacobiano traslazionale CoM coscia
Jc2 = jacobian([x_c2; y_c2], Q);

% Jacobiano traslazionale estremo link2 (meq)
Jh  = jacobian([xh; yh], Q);

% Vettori velocità (per verifica e uso esplicito)
v_c1 = Jc1 * dQ;   % [2x1]
v_c2 = Jc2 * dQ;
v_h  = Jh  * dQ;

%% 5. Energie cinetiche

% Ruota: traslazione + rotazione (centro fisso a quota Rw)
Tw   = 0.5 * mw * (Rw * dqw)^2 + 0.5 * Iw * dqw^2;

% Stinco: traslazione CoM + rotazione attorno a CoM
T1   = 0.5 * m1 * (v_c1' * v_c1) + 0.5 * I1 * dq1^2;

% Coscia: traslazione CoM + rotazione attorno a CoM
T2   = 0.5 * m2 * (v_c2' * v_c2) + 0.5 * I2 * dq2^2;

% Massa concentrata estremo link2 (pura traslazione, corpo puntiforme)
Tmeq = 0.5 * meq * (v_h' * v_h);

T = simplify(Tw + T1 + T2 + Tmeq);
disp('Energia cinetica calcolata.');

%% 6. Energia potenziale (quota zero = piano di appoggio ruota)

% mw*g*Rw è costante (non contribuisce alle EdM) ma lo lasciamo per correttezza
U = simplify(mw*g*Rw + m1*g*y_c1 + m2*g*y_c2 + meq*g*yh);
disp('Energia potenziale calcolata.');

%% 7. Equazioni di Eulero-Lagrange
% d/dt(∂L/∂dq_i) - ∂L/∂q_i = tau_i

L = T - U;

% Termine d/dt(∂L/∂dQ): applicando regola della catena
dLddQ     = jacobian(L, dQ);         % [1x3] gradiente rispetto a dQ
ddLddQ_dQ = jacobian(dLddQ', Q);     % [3x3] derivata rispetto a Q
ddLddQ_dQ_times_dQ = ddLddQ_dQ * dQ;% contributo da variazione di q

ddLddQ_ddQ = jacobian(dLddQ', dQ);   % [3x3] = matrice di massa M(q)

dL_dQ = jacobian(L, Q)';             % [3x1] gradiente rispetto a Q

% Equazioni del moto: M*ddQ + C*dQ + G = tau
Eq_moto = simplify(ddLddQ_dQ_times_dQ + ddLddQ_ddQ * ddQ - dL_dQ);

%% 8. Estrazione matrici dinamiche

% Matrice di massa M(q) [3x3] — simmetrica e definita positiva
M_matrix = simplify(jacobian(Eq_moto, ddQ));

% Vettore di gravità G(q) [3x1] — termini con dQ=0, ddQ=0
G_matrix = simplify(subs(Eq_moto, [dQ; ddQ], zeros(6,1)));

% Prodotto C(q,dq)*dq [3x1] — residuo (include termini di Coriolis e centrifughi)
C_times_dQ = simplify(Eq_moto - M_matrix * ddQ - G_matrix);

% --- Matrice C esplicita tramite simboli di Christoffel (opzionale ma corretta) ---
% C_{ij} = sum_k [ 1/2*(dM_ij/dq_k + dM_ik/dq_j - dM_jk/dq_i) ] * dq_k
n = length(Q);
C_matrix = sym(zeros(n,n));
for i = 1:n
    for j = 1:n
        for k = 1:n
            Gamma_ijk = 0.5 * ( diff(M_matrix(i,j), Q(k)) ...
                               + diff(M_matrix(i,k), Q(j)) ...
                               - diff(M_matrix(j,k), Q(i)) );
            C_matrix(i,j) = C_matrix(i,j) + Gamma_ijk * dQ(k);
        end
    end
end
C_matrix = simplify(C_matrix);
disp('Matrici dinamiche estratte.');

% Verifica: ||C*dQ - C_times_dQ|| deve essere zero
verifica = simplify(C_matrix * dQ - C_times_dQ);
if all(verifica == 0, 'all')
    disp('Verifica Christoffel: OK — C*dQ coerente con residuo Lagrangiano.');
else
    disp('ATTENZIONE: discrepanza tra C Christoffel e residuo. Controllare.');
end

%% 9. Matrice di attrito viscoso B [3x3] — diagonale
% Modello: tau_attrito = -B * dQ (dissipazione lineare in velocità)
% Ordine diagonale coerente con Q = [q1; qw; q2]
% Le EdM complete diventano: M*ddQ + (C+B)*dQ + G = tau_input
B_matrix = diag([b1; bw; b2]);
disp('Matrice di attrito viscoso B costruita.');

%% 10. Generazione file dinamica numerica

params_sym = [Rw, mw, m1, m2, meq, l1, l2, lc1, lc2, g, Iw, I1, I2, b1, bw, b2];
params_num = [robot_params.Rw,  robot_params.mw,  robot_params.m1,  ...
              robot_params.m2,  robot_params.meq, robot_params.l1,  ...
              robot_params.l2,  robot_params.lc1, robot_params.lc2, ...
              robot_params.g,   robot_params.Iw,  robot_params.I1,  ...
              robot_params.I2,  robot_params.b1,  robot_params.bw,  ...
              robot_params.b2];

matlabFunction(subs(M_matrix,     params_sym, params_num), ...
               subs(C_matrix,     params_sym, params_num), ...
               subs(C_times_dQ,   params_sym, params_num), ...
               subs(G_matrix,     params_sym, params_num), ...
               subs(B_matrix,     params_sym, params_num), ...
    'File',    'calc_dynamics', ...
    'Vars',    {Q, dQ}, ...
    'Outputs', {'M', 'C', 'CdQ', 'G', 'B'});

disp('File calc_dynamics.m generato con successo!');
disp('Output: M(q), C(q,dq), CdQ(q,dq), G(q), B');
disp('EdM complete: M*ddQ + (C+B)*dQ + G = tau_input');
disp('Ordine Q: [q1; qw; q2] — stinco, ruota, coscia');