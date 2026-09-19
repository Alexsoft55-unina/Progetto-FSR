%% =====================================================================
%  wbr_sim.m  --  Anello chiuso (RK4 passo fisso) + animazione
%  Prerequisito: eseguire prima wbr_build_model.m
% =====================================================================
clear; clc; load('robot_params.mat','p');

%% ---------- Guadagni ----------
K.KP = -0.22;   K.KI = -0.06;   K.TH_MAX = deg2rad(12);            % esterno (NEGATIVI)
K.c1 = 6.0;     K.ka = 3.5;     K.kb = 12.0;   K.eps1 = 0.01;      % super-twisting pitch
K.c2 = 12.0;    K.k2 = 60.0;    K.eta2 = 20.0; K.phi2 = 0.05;      % ginocchio
K.c3 = 12.0;    K.k3 = 60.0;    K.eta3 = 20.0; K.phi3 = 0.05;      % torso
K.q2_ref = -0.30;   K.q3_ref = 0.0;
K.TAU_MAX = [25; 120; 120];
K.mu = 0.8;   K.N_nom = (p.mw+p.m1+p.m2+p.m3)*p.g;   K.Rw = p.Rw;

%% ---------- Simulazione ----------
dt = 1e-3;  Tend = 20;  N = round(Tend/dt);
B  = [-1 0 -1; 1 0 0; 0 1 -1; 0 0 1];

X  = [pi/2 + 0.05; 0; K.q2_ref; 0;  0;0;0;0];    % perturbazione iniziale su q1
Iv = 0;  W = 0;

T      = zeros(N,1);
Qlog   = zeros(N,4);
TAUlog = zeros(N,3);
DGlog  = zeros(N,5);

for k = 1:N
    t = (k-1)*dt;
    v_ref = 0.6*(t >= 1.0);                      % gradino di velocita'

    [tau, dIv, dW, dg] = wbr_ctrl(X(1:4), X(5:8), v_ref, Iv, W, K);

    % RK4 sulla dinamica, ZOH sulla coppia (come in un'implementazione reale)
    k1 = wbr_dyn(X,           tau, B);
    k2 = wbr_dyn(X + dt/2*k1, tau, B);
    k3 = wbr_dyn(X + dt/2*k2, tau, B);
    k4 = wbr_dyn(X + dt  *k3, tau, B);
    X  = X + dt/6*(k1 + 2*k2 + 2*k3 + k4);

    Iv = Iv + dt*dIv;                            % Eulero sugli stati del controllore
    W  = W  + dt*dW;

    T(k)=t;  Qlog(k,:)=X(1:4).';  TAUlog(k,:)=tau.';  DGlog(k,:)=dg.';
end

fprintf('cond(A) max = %.2f  (se > 1e3 sei vicino a una singolarita'')\n', max(DGlog(:,5)));
fprintf('errore finale di velocita'' = %+.4f m/s\n', DGlog(end,4));

%% ---------- Animazione ----------
opts.speed  = 1;        % 1 = tempo reale, 2 = meta' velocita'
opts.fps    = 40;
opts.follow = true;
opts.record = 'none';   % 'gif' | 'mp4' per esportare
opts.file   = 'wbr_smc';
wbr_animate(T, Qlog, TAUlog, p, opts);

%% ---------- helper: dinamica diretta ----------
function dx = wbr_dyn(x, tau, B)
    [M, CdQ, G] = calc_dynamics(x(1:4), x(5:8));
    ddq = M \ (B*tau - CdQ - G);
    dx  = [x(5:8); ddq];
end
