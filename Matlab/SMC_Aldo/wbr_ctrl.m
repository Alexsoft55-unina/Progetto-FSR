function [tau, dIv, dW, dg] = wbr_ctrl(Q, dQ, v_ref, Iv, W, p, K)
%WBR_CTRL  Cascata (v_com -> theta_b*) + SMC su y = [theta_b; q1; q2].
%   q3 (torso) e' la coordinata NON attuata: nessun riferimento.

tau = zeros(3,1);  dW = 0;  dIv = 0;  dg = zeros(6,1);   % pre-dimensionamento codegen

B = [-1 1 0; 1 0 0; 1 0 1; -1 0 0];        % tau = [tau_w; tau_1(gin.); tau_2(anca)]

[M, CdQ, G]                 = calc_dynamics(Q, dQ);
[th, dth, J_th, h_th, v]    = wbr_kin(Q, dQ, p);

% ---- anello esterno (lento) --------------------------------------------
ev     = v - v_ref;
th_raw = K.KP*ev + K.KI*Iv;
th_ref = min(max(th_raw, -K.TH_MAX), K.TH_MAX);
dIv    = ev * (abs(th_raw - th_ref) < 1e-9);            % anti-windup

% ---- superfici ----------------------------------------------------------
Jy = [J_th; 1 0 0 0; 0 0 1 0];
hy = [h_th; 0; 0];

e  = [th - th_ref; Q(1) - K.q1_ref; Q(3) - K.q2_ref];
de = Jy*dQ;                                             % d(th_ref)/dt ~ 0
c  = [K.c1; K.c2; K.c3];
s  = de + c.*e;

% ---- leggi di scivolamento ---------------------------------------------
% canale 1: super-twisting (continuo, W e' lo stato integrale)
sg1 = s(1)/(abs(s(1)) + K.eps1);                        % sign regolarizzato
nu1 = -K.ka*sqrt(abs(s(1)))*sg1 + W;
dW  = -K.kb*sg1;

% canali 2-3: SMC classico + boundary layer
nu2 = -K.k2*sat_(s(2)/K.phi2) - K.eta2*s(2);
nu3 = -K.k3*sat_(s(3)/K.phi3) - K.eta3*s(3);
nu  = [nu1; nu2; nu3];

% ---- inversione ---------------------------------------------------------
Ad = Jy*(M\B);                                          % 3x3
bd = -Jy*(M\(CdQ + G)) + hy;

tau_raw = Ad \ ( -bd - c.*de + nu );

% ---- saturazioni --------------------------------------------------------
tau_slip   = K.mu * K.N_nom * K.Rw;                     % aderenza (N nominale!)
lim        = K.TAU_MAX;  lim(1) = min(lim(1), tau_slip);
tau        = min(max(tau_raw, -lim), lim);

if any(abs(tau_raw - tau) > 1e-9)
    dW = 0;                                             % congela il twisting in saturazione
end

% ---- diagnostica --------------------------------------------------------
sigma = M(2,2) + M(4,2);        % > 0 sempre in esercizio; ->0 = perdita autorita' ruota
dg = [th; th_ref; v; s(1); Q(4); sigma];
end

function y = sat_(x)
y = min(max(x, -1), 1);
end