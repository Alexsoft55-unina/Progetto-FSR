%% =====================================================================
%  wbr_build_model.m  --  Costruzione modello simbolico + generazione
%                         funzioni numeriche per Simulink
%
%  Robot bipede su ruote (rif. Cui et al., 2022)
%  Coordinate:  Q = [q1; qw; q2; q3]
%     q1 : angolo ASSOLUTO dello stinco (dall'orizzontale)   -> PASSIVO
%     qw : rotazione ASSOLUTA della ruota                    -> attivo (tau_w)
%     q2 : angolo RELATIVO di ginocchio (coscia = q1+q2)     -> attivo (tau_2)
%     q3 : angolo ASSOLUTO del torso (dalla verticale)       -> attivo (tau_3)
%
%  Esegui UNA VOLTA. Genera:
%     calc_dynamics.m -> [M, CdQ, G]      (4x4, 4x1, 4x1)
%     calc_kin.m      -> uscite + Jacobiane + termini di deriva
% =====================================================================
clear; clc;

%% ---------- 1. Parametri ----------
p.Rw = 0.127;  p.mw = 3.5;   p.Iw = 0.1;
p.m1 = 1.2;    p.m2 = 5.3;   p.m3 = 60.0;
p.l1 = 0.45;   p.l2 = 0.45;  p.l3 = 0.35;
p.g  = 9.81;
p.I1 = (1/12)*p.m1*p.l1^2;
p.I2 = (1/12)*p.m2*p.l2^2;
p.I3 = (1/12)*p.m3*p.l3^2;      % ipotesi asta sottile: DA RIVEDERE per un torso di 60 kg
save('robot_params.mat','p');

%% ---------- 2. Simboli ----------
syms q1 qw q2 q3  dq1 dqw dq2 dq3  real
Q  = [q1; qw; q2; q3];
dQ = [dq1; dqw; dq2; dq3];

Rw=p.Rw; mw=p.mw; Iw=p.Iw; m1=p.m1; m2=p.m2; m3=p.m3;
l1=p.l1; l2=p.l2; l3=p.l3; g=p.g; I1=p.I1; I2=p.I2; I3=p.I3;

%% ---------- 3. Cinematica dei baricentri ----------
sw = Rw*qw;                                   % vincolo di rotolamento puro
x1 = l1/2*cos(q1) + sw;                                y1 = l1/2*sin(q1);
x2 = l1*cos(q1) + l2/2*cos(q1+q2) + sw;                y2 = l1*sin(q1) + l2/2*sin(q1+q2);
x3 = l1*cos(q1) + l2*cos(q1+q2) + l3/2*sin(q3) + sw;   y3 = l1*sin(q1) + l2*sin(q1+q2) + l3/2*cos(q3);

v1 = [jacobian(x1,Q)*dQ; jacobian(y1,Q)*dQ];
v2 = [jacobian(x2,Q)*dQ; jacobian(y2,Q)*dQ];
v3 = [jacobian(x3,Q)*dQ; jacobian(y3,Q)*dQ];

%% ---------- 4. Energie e Lagrangiana ----------
T = 0.5*mw*(Rw*dqw)^2 + 0.5*Iw*dqw^2 ...
  + 0.5*m1*(v1.'*v1) + 0.5*I1*dq1^2 ...
  + 0.5*m2*(v2.'*v2) + 0.5*I2*(dq1+dq2)^2 ...
  + 0.5*m3*(v3.'*v3) + 0.5*I3*dq3^2;
U = mw*g*Rw + m1*g*(Rw+y1) + m2*g*(Rw+y2) + m3*g*(Rw+y3);
L = T - U;

dLddq = jacobian(L,dQ).';
M   = simplify(jacobian(dLddq, dQ));           % 4x4
CdQ = simplify(jacobian(dLddq, Q)*dQ);         % 4x1  (= C(q,dq)*dq)
Gv  = simplify(jacobian(U, Q).');              % 4x1

%% ---------- 5. Uscite controllate ----------
mb = m1+m2+m3;  mt = mb + mw;
xc = (m1*x1 + m2*x2 + m3*x3)/mb;               % CoM upper body
yc = (m1*y1 + m2*y2 + m3*y3)/mb;
th = atan2(xc - sw, yc);                       % y1: angolo CoM upper body dalla verticale
hb = sqrt((xc-sw)^2 + yc^2);                   % lunghezza pendolo equivalente (solo monitor!)
xG = (mw*sw + m1*x1 + m2*x2 + m3*x3)/mt;       % CoM totale
vG = jacobian(xG,Q)*dQ;                        % y2: velocita' traslazionale CoM totale

% Jacobiane e termini di deriva:   ddot(y) = J*ddQ + a
Jth = jacobian(th, Q);        ath = jacobian(Jth*dQ, Q)*dQ;
Jv  = jacobian(vG, dQ);       av  = jacobian(vG, Q)*dQ;
dth = Jth*dQ;

%% ---------- 6. Generazione file ----------
matlabFunction(M, CdQ, Gv, 'File','calc_dynamics', ...
    'Vars',{Q,dQ}, 'Outputs',{'M','CdQ','G'});

matlabFunction(th, dth, vG, hb, Jth, ath, Jv, av, 'File','calc_kin', ...
    'Vars',{Q,dQ}, 'Outputs',{'th','dth','vG','hb','Jth','ath','Jv','av'});

disp('Generati: calc_dynamics.m, calc_kin.m, robot_params.mat');

%% ---------- 7. Diagnostica: condizionamento della matrice di disaccoppiamento ----------
% Verifica che g_theta,w non si annulli e che A sia ben condizionata.
fprintf('\n  q1[deg] q2[deg] q3[deg] | g_th,w    g_v,w    cond(A)\n');
for a1 = deg2rad([70 90 110])
  for a2 = deg2rad([-20 0 20])
    for a3 = deg2rad([-20 0 20])
      z = [a1;0;a2;a3];  dz = zeros(4,1);
      [~,~,fth,gth,fv,gv,D,Mbar,hbar] = wbr_terms(z,dz);   %#ok<ASGLU>
      A = [gth.'; 0 1 0; 0 0 1];
      fprintf('  %6.0f %6.0f %6.0f | %+8.4f %+8.4f  %7.2f\n', ...
              rad2deg(a1),rad2deg(a2),rad2deg(a3), gth(1), gv(1), cond(A));
    end
  end
end
