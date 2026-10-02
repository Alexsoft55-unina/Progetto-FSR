"""
Modello Lagrangiano planare di RoTino con la struttura di Matlab/SMC/init.m (Cui et al. 2022).

Coordinate Q = [q1, qw, q2, q3] come in init.m:
    q1 : angolo ASSOLUTO dello stinco dall'orizzontale (antiorario)      -> passivo
    qw : rotazione ASSOLUTA della ruota (positiva in avanti, s_w = r qw)  -> tau_w
    q2 : angolo RELATIVO di ginocchio (coscia = q1 + q2)                 -> tau_2
    q3 : beccheggio ASSOLUTO del torso dalla verticale (+ in avanti)     -> tau_3

Differenze rispetto a init.m:
  - parametri letti dall'URDF di RoTino; le due gambe sono sommate (piano sagittale);
  - il baricentro del torso (base + zavorra) e' a (a3, b3) nella terna del torso e non a l3/2
    sopra l'anca (RoTino ha il baricentro del torso quasi alla quota dell'anca);
  - B ricavata dal lavoro virtuale degli angoli relativi dei motori. La B di init.m ha segni
    incoerenti con le sue stesse convenzioni (vedi check_B).

Uscite: stampa delle verifiche e modello.json (LaTeX simbolico + valori numerici) per il deck.
Eseguire con l'ambiente ROS e il workspace sourced.
"""

import json
import math
import os

import numpy as np
import sympy as sp

from rotino_description.model import WBRModel, load_urdf_from_package

HERE = os.path.dirname(os.path.abspath(__file__))

# ------------------------------------------------------------------ simboli
q1, qw, q2, q3 = sp.symbols('q_1 q_w q_2 q_3', real=True)
dq1, dqw, dq2, dq3 = sp.symbols(r'\dot{q}_1 \dot{q}_w \dot{q}_2 \dot{q}_3', real=True)
Q = sp.Matrix([q1, qw, q2, q3])
dQ = sp.Matrix([dq1, dqw, dq2, dq3])
m1, m2, m3, mw, I1, I2, I3, Iw = sp.symbols('m_1 m_2 m_3 m_w I_1 I_2 I_3 I_w', positive=True)
l1, l2, r, g = sp.symbols('l_1 l_2 r g', positive=True)
a3, b3 = sp.symbols('a_3 b_3', real=True)
PARAMS = (m1, m2, m3, mw, I1, I2, I3, Iw, l1, l2, r, g, a3, b3)


def build_model():
    sw = r * qw                                            # puro rotolamento
    # baricentri (x avanti, y in alto, origine sul centro ruota a quota r)
    x1, y1 = l1 / 2 * sp.cos(q1) + sw, l1 / 2 * sp.sin(q1)
    xk, yk = l1 * sp.cos(q1) + sw, l1 * sp.sin(q1)          # ginocchio
    x2, y2 = xk + l2 / 2 * sp.cos(q1 + q2), yk + l2 / 2 * sp.sin(q1 + q2)
    xh, yh = xk + l2 * sp.cos(q1 + q2), yk + l2 * sp.sin(q1 + q2)   # anca
    # torso: asse x del corpo = (cos q3, -sin q3), asse z del corpo = (sin q3, cos q3)
    x3 = xh + a3 * sp.cos(q3) + b3 * sp.sin(q3)
    y3 = yh - a3 * sp.sin(q3) + b3 * sp.cos(q3)

    def vel(x, y):
        return sp.Matrix([(sp.Matrix([x]).jacobian(Q) * dQ)[0], (sp.Matrix([y]).jacobian(Q) * dQ)[0]])

    v1, v2, v3 = vel(x1, y1), vel(x2, y2), vel(x3, y3)
    T = (sp.Rational(1, 2) * mw * (r * dqw) ** 2 + sp.Rational(1, 2) * Iw * dqw ** 2
         + sp.Rational(1, 2) * m1 * (v1.T * v1)[0] + sp.Rational(1, 2) * I1 * dq1 ** 2
         + sp.Rational(1, 2) * m2 * (v2.T * v2)[0] + sp.Rational(1, 2) * I2 * (dq1 + dq2) ** 2
         + sp.Rational(1, 2) * m3 * (v3.T * v3)[0] + sp.Rational(1, 2) * I3 * dq3 ** 2)
    U = mw * g * r + m1 * g * (r + y1) + m2 * g * (r + y2) + m3 * g * (r + y3)

    M = sp.simplify(sp.hessian(T, dQ))
    G = sp.simplify(sp.Matrix([U]).jacobian(Q).T)
    # C con i simboli di Christoffel di prima specie
    n = 4
    C = sp.zeros(n, n)
    for k in range(n):
        for j in range(n):
            C[k, j] = sp.simplify(sum(sp.Rational(1, 2) * (sp.diff(M[k, j], Q[i]) + sp.diff(M[k, i], Q[j])
                                                          - sp.diff(M[i, j], Q[k])) * dQ[i] for i in range(n)))
    CdQ = sp.simplify(C * dQ)

    # B dal lavoro virtuale: tau * d(angolo relativo del motore)/dQ, angoli misurati nello stesso verso
    phi_w = qw + q1             # ruota rispetto allo stinco, verso orario (= avanzamento)
    phi_k = q2                  # ginocchio relativo
    phi_h = q3 + q1 + q2        # torso rispetto alla coscia, verso orario
    B = sp.Matrix([[sp.diff(p, qi) for p in (phi_w, phi_k, phi_h)] for qi in Q])

    com = {'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2, 'x3': x3, 'y3': y3, 'xh': xh, 'yh': yh}
    return M, C, CdQ, G, B, T, U, com


def rotino_numbers():
    model = WBRModel(load_urdf_from_package())
    p = model.p
    links = {l.name: l for l in model.robot.links}
    base, ballast = links['base_link'].inertial, links['ballast_link'].inertial
    c_base = np.array([0.025, 0.0])                  # (x, z) nella terna del torso
    c_ball = np.array([0.045, -0.045])
    a, b = p.torso_com[0], p.torso_com[2]
    I3v = (base.inertia.iyy + ballast.inertia.iyy
           + base.mass * np.sum((c_base - [a, b]) ** 2) + ballast.mass * np.sum((c_ball - [a, b]) ** 2))
    num = {
        m1: 2 * p.m_1, m2: 2 * p.m_2, m3: p.m_3, mw: 2 * p.m_w,
        I1: 2 * links['left_shank_link'].inertial.inertia.iyy,
        I2: 2 * links['left_thigh_link'].inertial.inertia.iyy,
        I3: I3v, Iw: 2 * p.I_w,
        l1: p.l_1, l2: p.l_2, r: p.r, g: 9.81, a3: a, b3: b,
    }
    return model, num


def check_B(B):
    """Un moto rigido dell'intero robot (tutti i corpi ruotano di delta in verso antiorario) non
    cambia nessun angolo relativo: le colonne di B devono essere ortogonali a quel moto."""
    rigid = sp.Matrix([1, -1, 0, -1])   # dq1 = +d, dqw = -d (orario +), dq2 = 0, dq3 = -d (orario +)
    ours = [(B[:, j].T * rigid)[0] for j in range(3)]
    init_m = sp.Matrix([[-1, 0, -1], [1, 0, 0], [0, 1, -1], [0, 0, 1]])
    theirs = [(init_m[:, j].T * rigid)[0] for j in range(3)]
    return ours, theirs, init_m


def main():
    M, C, CdQ, G, B, T, U, com = build_model()
    model, num = rotino_numbers()

    # ---------------- posa nominale: tutte le giunture URDF a zero, torso orizzontale
    q_nom = {q1: 3 * sp.pi / 4, qw: 0, q2: -sp.pi / 2, q3: 0}
    dq_zero = {d: 0 for d in dQ}
    Mn = np.array(M.subs(num).subs(q_nom).evalf(), dtype=float)
    Gn = np.array(G.subs(num).subs(q_nom).evalf(), dtype=float).ravel()
    eig = np.linalg.eigvalsh(Mn)

    # ---------------- verifiche
    assert np.allclose(Mn, Mn.T), 'M non simmetrica'
    assert np.all(eig > 0), 'M non definita positiva'
    total = sum(float(num[s]) for s in (m1, m2, m3, mw))
    assert abs(total - model.kin.total_mass) < 1e-9, total
    # baricentro del corpo superiore rispetto all'asse, confronto con WBRModel
    mb = m1 + m2 + m3
    xc = (m1 * com['x1'] + m2 * com['x2'] + m3 * com['x3']) / mb - r * qw
    yc = (m1 * com['y1'] + m2 * com['y2'] + m3 * com['y3']) / mb
    S_C = float(xc.subs(num).subs(q_nom))
    Z_C = float(yc.subs(num).subs(q_nom))
    e = model.equivalent_centroid(0.0, 0.0)
    assert abs(S_C - e['S_C']) < 1e-6 and abs(Z_C - e['Z_C']) < 1e-6, (S_C, Z_C, e)
    # passivita': Mdot - 2C antisimmetrica (su uno stato qualunque)
    Mdot = sp.zeros(4, 4)
    for i in range(4):
        for j in range(4):
            Mdot[i, j] = sum(sp.diff(M[i, j], Q[k]) * dQ[k] for k in range(4))
    N = (Mdot - 2 * C).subs(num)
    test = {q1: 2.1, qw: 0.3, q2: -1.4, q3: 0.2, dq1: 0.7, dqw: -1.1, dq2: 0.4, dq3: -0.9}
    Nn = np.array(N.subs(test).evalf(), dtype=float)
    assert np.allclose(Nn, -Nn.T), 'Mdot - 2C non antisimmetrica'
    # B
    ours, theirs, B_init = check_B(B)
    assert all(v == 0 for v in ours), ours
    # coerenza: C*dQ dai Christoffel == termine di Coriolis di init.m (jacobian(dL/ddq, q) dq - dL/dq)
    L = T - U
    dLddq = sp.Matrix([L]).jacobian(dQ).T
    CdQ_init = dLddq.jacobian(Q) * dQ - sp.Matrix([T]).jacobian(Q).T
    diff = sp.simplify((CdQ_init - CdQ).subs(num).subs(test))
    assert all(abs(float(x)) < 1e-9 for x in diff), diff

    # ---------------- forma compatta per le slide, verificata contro quella estesa
    k1 = (m1 / 2 + m2 + m3) * l1                 # momento statico della catena sopra la ruota, su l1
    k2 = (m2 / 2 + m3) * l2                      # idem sopra il ginocchio, su l2
    J2 = I2 + (m2 / 4 + m3) * l2 ** 2
    J1 = I1 + (m1 / 4 + m2 + m3) * l1 ** 2

    def psi(a):                                  # proiezione del baricentro del torso
        return a3 * sp.cos(a) + b3 * sp.sin(a)

    def dpsi(a):
        return -a3 * sp.sin(a) + b3 * sp.cos(a)

    s1, s12, c1, c12 = sp.sin(q1), sp.sin(q1 + q2), sp.cos(q1), sp.cos(q1 + q2)
    q13, q123 = q1 + q3, q1 + q2 + q3
    Mc = sp.Matrix([
        [J1 + J2 + 2 * l1 * k2 * sp.cos(q2), -r * (k1 * s1 + k2 * s12), J2 + l1 * k2 * sp.cos(q2),
         -m3 * (l1 * psi(q13) + l2 * psi(q123))],
        [0, Iw + (mw + m1 + m2 + m3) * r ** 2, -r * k2 * s12, m3 * r * dpsi(q3)],
        [0, 0, J2, -m3 * l2 * psi(q123)],
        [0, 0, 0, I3 + m3 * (a3 ** 2 + b3 ** 2)]])
    for i in range(4):
        for j in range(i):
            Mc[i, j] = Mc[j, i]
    CdQc = sp.Matrix([
        -l1 * k2 * sp.sin(q2) * (2 * dq1 * dq2 + dq2 ** 2) - m3 * dq3 ** 2 * (l1 * dpsi(q13) + l2 * dpsi(q123)),
        -r * (k1 * c1 * dq1 ** 2 + k2 * c12 * (dq1 + dq2) ** 2 + m3 * psi(q3) * dq3 ** 2),
        l1 * k2 * sp.sin(q2) * dq1 ** 2 - m3 * l2 * dpsi(q123) * dq3 ** 2,
        -m3 * (l1 * dpsi(q13) * dq1 ** 2 + l2 * dpsi(q123) * (dq1 + dq2) ** 2)])
    Gc = sp.Matrix([g * (k1 * c1 + k2 * c12), 0, g * k2 * c12, -g * m3 * psi(q3)])
    for full, compact, name in ((M, Mc, 'M'), (CdQ, CdQc, 'CdQ'), (G, Gc, 'G')):
        assert sp.simplify(sp.expand(full - compact)) == sp.zeros(*full.shape), name
    compact_num = {name: float(expr.subs(num)) for name, expr in
                   (('k1', k1), ('k2', k2), ('J1', J1), ('J2', J2), ('m3a3', m3 * a3), ('m3b3', m3 * b3),
                    ('M22', Iw + (mw + m1 + m2 + m3) * r ** 2), ('M44', I3 + m3 * (a3 ** 2 + b3 ** 2)))}

    # ---------------- partizione passivo/attivo (wbr_terms.m) in posa nominale
    Bn = np.array(B, dtype=float)
    Muu, Mua, Mau, Maa = Mn[0, 0], Mn[0, 1:], Mn[1:, 0], Mn[1:, 1:]
    D = Bn[1:] - np.outer(Mau / Muu, Bn[0])
    Mbar = Maa - np.outer(Mau, Mua) / Muu

    # ---------------- LaTeX
    def tex(expr):
        return sp.latex(sp.trigsimp(sp.factor_terms(sp.expand(expr))), mul_symbol=' ')

    out = {
        'params': {str(k): float(v) for k, v in num.items()},
        'q_nom_deg': {'q1': 135.0, 'qw': 0.0, 'q2': -90.0, 'q3': 0.0},
        'M_tex': [[tex(M[i, j]) for j in range(4)] for i in range(4)],
        'CdQ_tex': [tex(CdQ[i]) for i in range(4)],
        'G_tex': [tex(G[i]) for i in range(4)],
        'B': Bn.tolist(), 'B_init_m': np.array(B_init, dtype=float).tolist(),
        'B_check_ours': [int(v) for v in ours], 'B_check_init_m': [int(v) for v in theirs],
        'M_nom': Mn.tolist(), 'G_nom': Gn.tolist(), 'M_eig': eig.tolist(),
        'D_nom': D.tolist(), 'Mbar_nom': Mbar.tolist(),
        'S_C': S_C, 'Z_C': Z_C, 'total_mass': total, 'compact': compact_num,
        'com_tex': {k: sp.latex(v) for k, v in com.items()},
    }
    with open(os.path.join(HERE, 'modello.json'), 'w') as f:
        json.dump(out, f, indent=1)

    np.set_printoptions(precision=5, suppress=True)
    print('Parametri (gambe sommate):', {k: round(v, 6) for k, v in out['params'].items()})
    print('M(q_nom) =\n', Mn)
    print('autovalori M:', eig)
    print('G(q_nom) =', Gn)
    print('B =\n', Bn)
    print('verifica moto rigido: B nostra', ours, ' B di init.m', theirs)
    print(f'S_C = {S_C:.5f}  Z_C = {Z_C:.5f}  (WBRModel: {e["S_C"]:.5f}, {e["Z_C"]:.5f})')
    print('D =\n', D, '\nMbar =\n', Mbar)
    print('coefficienti della forma compatta:', {k: round(v, 6) for k, v in compact_num.items()})
    print('Tutte le verifiche superate. Scritto modello.json')


if __name__ == '__main__':
    main()
