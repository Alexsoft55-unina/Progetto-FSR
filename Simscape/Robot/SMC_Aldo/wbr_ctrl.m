% invece di calcolare th, v da wbr_kin, li ricevi come ingressi misurati:
function [tau, dIv, dW, dg,th_ref, errors] = wbr_ctrl(Q,Q_ref, dQ, th_meas, dth_meas, v_meas, v_ref, Iv, W, p, K,B)
[M, CdQ, G]           = calc_dynamics(Q, dQ);
[~, ~, J_th, h_th, ~] = wbr_kin(Q, dQ, p);   % SERVONO ANCORA per Jy, hy, Ad, bd

dW = zeros(3, 1);

q1_ref = Q_ref(1); %Ginocchio
q2_ref = Q_ref(2); %Anca

ev     = v_meas - v_ref;     

% ---- anello esterno (lento) --------------------------------------------
th_raw = K.KP*ev + K.KI*Iv;
th_ref = min(max(th_raw, -K.TH_MAX), K.TH_MAX);
dIv    = ev * (abs(th_raw - th_ref) < 1e-9);            % anti-windup
%th_ref=0.0;
% ---- superfici ----------------------------------------------------------
Jy = [J_th; 1 0 0 0; 0 0 1 0];
hy = [h_th; 0; 0];

e  = [th_meas - th_ref; Q(1)-q1_ref; Q(3)-q2_ref];  
de = [dth_meas; dQ(1); dQ(3)];

c  = [K.c1; K.c2; K.c3];
s  = de + c.*e;

% ---- leggi di scivolamento ---------------------------------------------
% canale 1: super-twisting (continuo, W(1) e' lo stato integrale)
sg1 = s(1)/(abs(s(1)) + K.eps1);                        % sign regolarizzato
nu1 = -K.ka*sqrt(abs(s(1)))*sg1 + W(1);
dW1 = -K.kb*sg1;

% canali 2-3: SMC classico + boundary layer + Azione Integrale
% Si integra la funzione sat_() per ridurre chattering e migliorare la convergenza
nu2 = -K.k2*sat_(s(2)/K.phi2) - K.eta2*s(2) + W(2);
dW2 = -K.ki2 * sat_(s(2)/K.phi2); 

nu3 = -K.k3*sat_(s(3)/K.phi3) - K.eta3*s(3) + W(3);
dW3 = -K.ki3 * sat_(s(3)/K.phi3);

nu  = [nu1; nu2; nu3];
dW  = [dW1; dW2; dW3]; % Ora dW è un vettore 3x1

% ---- inversione ---------------------------------------------------------
Ad = Jy*(M\B);                                          % 3x3
bd = -Jy*(M\(CdQ + G)) + hy;

tau_raw = Ad \ ( -bd - c.*de + nu );

% ---- saturazioni --------------------------------------------------------
tau_slip   = K.TAU_MAX(1);K.mu * K.N_nom * K.Rw;                     % aderenza (N nominale!)
lim        = K.TAU_MAX;  
lim(1) = min(lim(1), tau_slip);
tau        = min(max(tau_raw, -lim), lim);

if any(abs(tau_raw - tau) > 1e-9*1e9)
    dW(:) = 0;                                             % congela il twisting in saturazione
end

% ---- diagnostica --------------------------------------------------------
sigma  = M(2,2) + M(4,2);
detAd  = det(Ad);
condAd = cond(Ad);
dg = [sigma; detAd; condAd];

errors  = e;
end

function y = sat_(x)
y = min(max(x, -1), 1);
end