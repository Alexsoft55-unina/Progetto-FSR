function [th, dth, J_th, h_th, v, J_v, h_v, SC, ZC] = wbr_kin(Q, dQ, p)
% Q = [q1(ginocchio); qw(ruota); q2(anca); q3(torso assoluto)]
q1 = Q(1); q2 = Q(3); q3 = Q(4);
dq1 = dQ(1); dq2 = dQ(3); dq3 = dQ(4);

mb  = p.m1 + p.m2 + p.m3;   mt = mb + p.mw;   kap = mb/mt;

Ca = (p.m1*p.l1/2 + (p.m2+p.m3)*p.l1)/mb;
Cb = (p.m2*p.l2/2 +  p.m3*p.l2)/mb;
Cc = (p.m3*p.l3/2)/mb;

a  = q1 - q2 + q3;   da = dq1 - dq2 + dq3;   % stinco (= caviglia passiva)
b  = q3 - q2;        db = dq3 - dq2;         % coscia
c  = q3;             dc = dq3;               % torso

sa=sin(a); ca=cos(a); sb=sin(b); cb=cos(b); sc=sin(c); cc=cos(c);

SC = Ca*sa + Cb*sb + Cc*sc;      % CoM corpo, orizzontale, rel. mozzo
ZC = Ca*ca + Cb*cb + Cc*cc;      % CoM corpo, verticale,   rel. mozzo

J_S = [ Ca*ca, 0, -Ca*ca - Cb*cb,  Ca*ca + Cb*cb + Cc*cc];
J_Z = [-Ca*sa, 0,  Ca*sa + Cb*sb, -Ca*sa - Cb*sb - Cc*sc];
h_S = -Ca*sa*da^2 - Cb*sb*db^2 - Cc*sc*dc^2;
h_Z = -Ca*ca*da^2 - Cb*cb*db^2 - Cc*cc*dc^2;

dS = J_S*dQ;  dZ = J_Z*dQ;  D = SC^2 + ZC^2;

th   = atan2(SC, ZC);                       % 0 = CoM sopra il mozzo
J_th = (ZC*J_S - SC*J_Z)/D;
dth  = J_th*dQ;
h_th = (ZC*h_S - SC*h_Z)/D - 2*dth*(SC*dS + ZC*dZ)/D;

J_v = [kap*J_S(1), p.Rw, kap*J_S(3), kap*J_S(4)];  % x_com = Rw*qw + kap*SC
v   = J_v*dQ;
h_v = kap*h_S;
end