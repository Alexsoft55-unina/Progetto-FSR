function [th, dth, fth, gth, fv, gv, D, Mbar, hbar] = wbr_terms(Q, dQ)
%WBR_TERMS  Partial Feedback Linearization NON collocata + uscite in forma affine.
%
%   Restituisce, per lo stato corrente:
%     th, dth   angolo del CoM dell'upper body e sua derivata
%     fth, gth  ddot(theta_b) = fth + gth'*v          (v = [ddq_w; ddq_2; ddq_3])
%     fv,  gv   dot(v_com)    = fv  + gv'*v
%     D,Mbar,hbar   per la mappa tau = D \ (Mbar*v + hbar)
%
%   Codegen-safe: nessun toolbox simbolico a runtime.

%--- matrice di ingresso (tau = [tau_w; tau_2; tau_3]) -------------------
% Riga q1 (passivo) NON nulla: reazione del motore ruota sullo stinco e
% del motore d'anca sulla catena. Sistema NON collocato.
B = [-1  0 -1;
      1  0  0;
      0  1 -1;
      0  0  1];

[M, CdQ, G] = calc_dynamics(Q, dQ);
h = CdQ + G;

%--- partizione passivo (u = 1) / attivo (a = 2:4) -----------------------
Muu = M(1,1);        Mua = M(1,2:4);      % 1x1, 1x3
Mau = M(2:4,1);      Maa = M(2:4,2:4);    % 3x1, 3x3
hu  = h(1);          ha  = h(2:4);
Bu  = B(1,:);        Ba  = B(2:4,:);

D    = Ba  - (Mau/Muu)*Bu;                % 3x3  -> invertibile
Mbar = Maa - (Mau/Muu)*Mua;               % complemento di Schur, def. pos.
hbar = ha  - (Mau/Muu)*hu;

Dinv = D \ eye(3);
bvec = (Bu*Dinv*Mbar - Mua)/Muu;          % 1x3 : ddq1 = acc + bvec*v
acc  = (Bu*Dinv*hbar - hu)/Muu;           % 1x1

P0 = [acc; 0; 0; 0];                      % ddQ = P0 + P1*v
P1 = [bvec; eye(3)];

%--- uscite in forma affine ---------------------------------------------
[th, dth, ~, ~, Jth, ath, Jv, av] = calc_kin(Q, dQ);

fth = Jth*P0 + ath;      gth = (Jth*P1).';
fv  = Jv *P0 + av;       gv  = (Jv *P1).';
end
