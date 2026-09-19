function [tau, dIv, dW, diag] = wbr_ctrl(Q, dQ, v_ref, Iv, W, K)
%WBR_CTRL  Controllore in cascata + Sliding Mode Control (SMC).
%
%   Architettura:
%     - anello ESTERNO (lento) : PI sull'errore di velocita' del CoM totale
%                                -> genera il riferimento di pitch theta_b*
%                                (guadagni NEGATIVI: sistema a fase non minima)
%     - anello INTERNO (veloce): super-twisting SMC su theta_b   (ruota, tau_w)
%     - postura                : SMC classico su q2 e q3         (ginocchio, torso)
%
%   Stati del controllore da integrare esternamente:
%     Iv : integrale dell'errore di velocita'   -> dIv
%     W  : stato interno del super-twisting     -> dW
%
%   Blocco Simulink: MATLAB Function, con Iv e W come Unit Delay / Integrator.
%
%   K : struct di guadagni (vedi wbr_gains).

%======================= 1. Uscite e forma affine ========================
[th, dth, fth, gth, fv, gv, D, Mbar, hbar] = wbr_terms(Q, dQ);   %#ok<ASGLU>
[~, ~, vG] = calc_kin(Q, dQ);

%======================= 2. Anello esterno: velocita' ====================
ev  = vG - v_ref;                                  % errore di velocita'
th_ref = K.KP*ev + K.KI*Iv;                        % KP,KI < 0  (fase non minima)
th_ref = max(-K.TH_MAX, min(K.TH_MAX, th_ref));    % saturazione: NON negoziabile
dIv = ev;
% anti-windup
if abs(K.KP*ev + K.KI*Iv) > K.TH_MAX,  dIv = 0;  end

%======================= 3. Anello interno: pitch (super-twisting) =======
e1  = th - th_ref;
de1 = dth;                                          % dth_ref ~ 0 (esterno lento)
s1  = de1 + K.c1*e1;

sg1 = tanh(s1/K.eps1);                              % sostituto continuo di sign()
w_th = -K.c1*de1 - K.ka*sqrt(abs(s1))*sg1 + W;      % ddot(theta_b) desiderata
dW   = -K.kb*sg1;                                   % integratore del super-twisting

%======================= 4. Postura: ginocchio e torso ===================
% NOTA: la terza uscita e' q2 (angolo di ginocchio), NON la lunghezza di gamba h_b.
% h_b e' SINGOLARE a q2 = 0 (gamba tesa, l1 = l2): cond(A) ~ 1e16.
e2 = Q(3) - K.q2_ref;  d2 = dQ(3);   s2 = d2 + K.c2*e2;
e3 = Q(4) - K.q3_ref;  d3 = dQ(4);   s3 = d3 + K.c3*e3;

w_2 = -K.c2*d2 - K.k2*s2 - K.eta2*tanh(s2/K.phi2);
w_3 = -K.c3*d3 - K.k3*s3 - K.eta3*tanh(s3/K.phi3);

%======================= 5. Disaccoppiamento e coppie ====================
A   = [gth.'; 0 1 0; 0 0 1];                        % cond(A) ~ 8.5, uniforme
rhs = [w_th - fth; w_2; w_3];
v   = A \ rhs;                                      % v = [ddq_w; ddq_2; ddq_3]

tau = D \ (Mbar*v + hbar);

% saturazione attuatori + limite di aderenza sulla ruota
tau = max(-K.TAU_MAX, min(K.TAU_MAX, tau));
tau_slip = K.mu * K.N_nom * K.Rw;                   % |tau_w| <= mu * N * Rw
tau(1) = max(-tau_slip, min(tau_slip, tau(1)));

diag = [th; th_ref; s1; ev; cond(A)];
end
