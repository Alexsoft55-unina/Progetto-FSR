clear all
close all
clc

syms x1 x2 x3 u1 u2 real 
A1=1;
A2=1;
A3=1;
a12=0.6;
a23=0.4;
a2=0.3;
a3=0.25;

x = [x1 x2 x3]';
u = [u1 u2]';
x1dot= (u1 - a12*sqrt(x1-x2))/A1;
x2dot = (a12*sqrt(x1-x2)-a23*sqrt(x2-x3)-a2*sqrt(x2))/A2;
x3dot = (u2 + a23*sqrt(x2-x3)- a3*sqrt(x3))/A3;

x_eq = [3.274 2 1.6]';
f = [x1dot x2dot x3dot]';
f_eq = subs(f, x, x_eq);
f_eq = round(f_eq,4);
sol = solve(f_eq == 0, u);

% Estraiamo i valori numerici degli ingressi di equilibrio
u1_val = double(sol.u1)
u2_val = double(sol.u2)
vars = [x1, x2, x3, u1, u2];
vals = [x_eq(1), x_eq(2), x_eq(3), u1_val, u2_val];



%Modello linearizzato 
As = jacobian(f, x);
Bs = jacobian(f,u);
A = double(subs(As, vars, vals))
B = double(subs(Bs, vars, vals))

C=eye(3);
D=zeros(3,2);

%% Verifica stabilità punto di equilibrio
autovalori = eig(A);
if all(real(autovalori) < 0)
    disp('Il sistema è ASINTOTICAMENTE STABILE.');
else
    disp('Il sistema NON è asintoticamente stabile.');
end

%% verifica raggiungibilità
R = ctrb(A, B);
rango_R = rank(R);
disp(['Rango della matrice di raggiungibilità: ', num2str(rango_R)]);

if rango_R == size(A, 1)
    disp('Il sistema è COMPLETAMENTE RAGGIUNGIBILE.');
else
    disp('Il sistema NON è raggiungibile.');
end

%% verifica osservabilità

O = obsv(A, C);
rango_O = rank(O);
disp(['Rango della matrice di osservabilità: ', num2str(rango_O)]);

if rango_O == size(A, 1)
    disp('Il sistema è COMPLETAMENTE OSSERVABILE.');
else
    disp('Il sistema NON è osservabile.');
end


%% place
eig_des = [-3 -2 -1];
K = -place(A,B,eig_des);
eig(A+B*K)

%% Compensazione guadagno 
sistema = ss(A+B*K,B,[1 0 0; 0 0 1],0);
ggg=dcgain(sistema);
K_com=inv(ggg)

%% LMI 
[n, m] = size(B);

fprintf('State-feedback LMI: n=%d, m=%d\n', n, m);

% Parametri LMI
epsLMI = 1e-6;

% Risolvo LMI per (W,N)
cvx_clear;
cvx_begin sdp quiet
    cvx_precision high
    variable Q(n,n) symmetric               %% Q=P^-1
    variable N(m,n)                         %% N=KQ
     minimize(0) % feasibility
     subject to
        Q >= epsLMI*eye(n);             %%vincolo: funzione candidata definita positiva
        A*Q + Q*A' + B*N + N'*B' <= -epsLMI*eye(n);     %%derivata funzione di Lyapunov definita negativa
cvx_end

if ~strcmp(cvx_status,'Solved')
    error('CVX non ha trovato soluzione per (W,N): %s', cvx_status);
end

Qval = Q;
Nval = N;
% ricostruisci K
K_LMI = Nval / Qval;

fprintf('K trovato (m x n):\n'); disp(K_LMI);
eig_cl = eig(A + B*K_LMI);
fprintf('Autovalori A+B*K:\n'); disp(eig_cl.');
fprintf('Costanti di tempo A+B*K:\n'); disp(-1./real(eig_cl)');

% Simulazione: risposta a condizione iniziale (solo per verificare stabilità)
t = 0:0.01:5;
x0 = [0.3 0.2 0.1];
% dinamica chiusa: xdot = (A + B*K)*x
sys_cl = ss(A + B*K_LMI, [], eye(n), []);
[y,t,x] = initial(sys_cl, x0, t);

figure('Name','State-feedback: stati (chiuso)');
for i=1:n
    subplot(n,1,i);
    plot(t, x(:,i), 'LineWidth', 1.4); grid on;
    ylabel(sprintf('x_%d', i));
    if i==1, title('Stati in anello chiuso con state-feedback da LMI'); end
end
xlabel('Tempo (s)');

fprintf('Esecuzione completata.\n');


sistema_LMI = ss(A+B*K_LMI,B,[1 0 0; 0 0 1],0);
ggg=dcgain(sistema_LMI);
K_comp_LMI=inv(ggg)


%% LQR
A_int = [A, zeros(3,1);-[0 1 0] 0];
B_int = [B; 0 0];
C_int = eye(4);
D_int = zeros(4,2);
% Scelta delle matrici di peso

Qtilde=1;
Q=C_int'*Qtilde*C_int;
R=1;

% problema LQ
sistema_int=ss(A_int, B_int, C_int,D_int)
[K_LQ,P,Poles] = lqr(sistema_int,Q,R)
K_LQ1=K_LQ(:,1:3);
K_LQint=K_LQ(:,4);

% A_ta=A+alfa*eye alfa=4.6/tad    formula per tempo di assestamento
alpha = 4.6;
A_ta = A_int + alpha*eye(4);
sistema_int_ta=ss(A_ta, B_int, C_int,D_int)
[K_LQ_ta,P,Poles] = lqr(sistema_int_ta,Q,R)
K_LQ1_ta=K_LQ_ta(:,1:3);
K_LQint_ta=K_LQ_ta(:,4);

%% LQG
Q_noise = 1e-4 * eye(4);  % Rumore di processo (4x4 perché abbiamo 4 stati)
R_noise = 1e-3 * eye(2);  % Rumore di misura (2x2 perché misuriamo x e theta)
D=zeros(2,4);
% Risoluzione dell'equazione di Riccati algebrica continua
[P,~,~] = care(A', C', Q_noise, R_noise);
L = P*C'*inv(R_noise);

A_obs = A - L*C;% Matrice di stato dell'osservatore
eig(A_obs)
B_obs = [B, L];           % Ingressi: [u (comando), y (misure carrello e pendolo)]
C_obs = eye(4);           % Uscita: vogliamo tutti i 4 stati stimati
D_obs = zeros(4, 3);      % 4 stati, 3 ingressi (1 comando u + 2 misure y)
Q_lqrg=diag([10, 1, 100, 1, 50]);
R_lqrg=0.1;
sys_lqri = ss(A_1,B_1,C_1,D_1);
[K_lqri,P_lq,PolesCL] = lqr(sys_lqri,Q_lqrg,R_lqrg);
display(K_lqri);
K_lqri1=K_lqri(:,1:3);
K_lqri2=K_lqri(:,4);
 
%H infinito

%Compensazione guadagno 
