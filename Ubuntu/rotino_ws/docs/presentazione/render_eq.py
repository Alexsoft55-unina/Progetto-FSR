"""Renderizza le formule della presentazione: LaTeX standalone -> PDF -> PNG trasparente.

Scrive eq/<nome>.png e eq/manifest.json con la dimensione naturale di ogni formula in pollici
(al corpo di FONT_PT), cosi' build_deck.js le inserisce senza deformarle.
"""

import json
import os
import shutil
import subprocess
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'eq')
FONT_PT = 12          # corpo di composizione; il deck scala con SCALE
DPI = 400

PREAMBLE = r"""\documentclass[border=2pt,12pt]{standalone}
\usepackage{amsmath,amssymb,bm}
\usepackage{xcolor}
\definecolor{acc}{HTML}{C0392B}
\definecolor{mpc}{HTML}{1F6F78}
\newcommand{\dd}{\mathrm{d}}
\begin{document}
$\displaystyle
"""

EQ = {
    # ------------------------------------------------------------------ 1. modello
    'cinematica': r"""
\begin{aligned}
s_w &= r\,q_w \quad\text{(puro rotolamento)}\\
p_1 &= \begin{bmatrix} s_w\\0\end{bmatrix} + \tfrac{l_1}{2}\begin{bmatrix}\cos q_1\\ \sin q_1\end{bmatrix},\qquad
p_k = \begin{bmatrix} s_w\\0\end{bmatrix} + l_1\begin{bmatrix}\cos q_1\\ \sin q_1\end{bmatrix}\\
p_2 &= p_k + \tfrac{l_2}{2}\begin{bmatrix}\cos(q_1{+}q_2)\\ \sin(q_1{+}q_2)\end{bmatrix},\qquad
p_h = p_k + l_2\begin{bmatrix}\cos(q_1{+}q_2)\\ \sin(q_1{+}q_2)\end{bmatrix}\\
p_3 &= p_h + a_3\begin{bmatrix}\cos q_3\\ -\sin q_3\end{bmatrix} + b_3\begin{bmatrix}\sin q_3\\ \cos q_3\end{bmatrix}
\end{aligned}""",
    'energie': r"""
\begin{aligned}
T &= \tfrac12\big(m_w r^2 + I_w\big)\dot q_w^2 + \sum_{i=1}^{3}\tfrac12 m_i\,\dot p_i^{T}\dot p_i
   + \tfrac12 I_1\dot q_1^2 + \tfrac12 I_2(\dot q_1{+}\dot q_2)^2 + \tfrac12 I_3\dot q_3^2\\[2pt]
U &= m_w g r + \sum_{i=1}^{3} m_i\,g\,(r + y_i), \qquad \mathcal{L} = T - U
\end{aligned}""",
    'eulero_lagrange': r"""
\frac{\dd}{\dd t}\frac{\partial\mathcal{L}}{\partial\dot q}-\frac{\partial\mathcal{L}}{\partial q}=B\,\tau
\;\;\Longrightarrow\;\;
\boxed{M(q)\,\ddot q + C(q,\dot q)\,\dot q + G(q) = B\,\tau}""",
    'definizioni_MCG': r"""
\begin{aligned}
M(q) &= \frac{\partial^2 T}{\partial \dot q\,\partial \dot q}, \qquad
G(q) = \frac{\partial U}{\partial q}^{T}\\
c_{kj} &= \sum_{i}\Gamma_{ijk}\,\dot q_i,\qquad
\Gamma_{ijk} = \tfrac12\Big(\frac{\partial M_{kj}}{\partial q_i}+\frac{\partial M_{ki}}{\partial q_j}-\frac{\partial M_{ij}}{\partial q_k}\Big)
\end{aligned}""",
    'M_compatta': r"""
M(q)=\begin{bmatrix}
J_1{+}J_2{+}2l_1k_2\cos q_2 & -r\,(k_1 s_1{+}k_2 s_{12}) & J_2{+}l_1k_2\cos q_2 & -m_3\big(l_1\psi_{13}{+}l_2\psi_{123}\big)\\
\star & I_w{+}(m_w{+}m_1{+}m_2{+}m_3)\,r^2 & -r\,k_2 s_{12} & m_3 r\,\psi'_{3}\\
\star & \star & J_2 & -m_3 l_2\,\psi_{123}\\
\star & \star & \star & I_3{+}m_3(a_3^2{+}b_3^2)
\end{bmatrix}""",
    'M_def': r"""
\begin{aligned}
k_1 &= \big(\tfrac{m_1}{2}{+}m_2{+}m_3\big)l_1, \quad k_2 = \big(\tfrac{m_2}{2}{+}m_3\big)l_2,\quad
J_1 = I_1{+}\big(\tfrac{m_1}{4}{+}m_2{+}m_3\big)l_1^2,\quad J_2 = I_2{+}\big(\tfrac{m_2}{4}{+}m_3\big)l_2^2\\
\psi(\alpha) &= a_3\cos\alpha + b_3\sin\alpha,\quad \psi'(\alpha) = -a_3\sin\alpha + b_3\cos\alpha,\quad
s_{12}=\sin(q_1{+}q_2),\ \psi_{123}=\psi(q_1{+}q_2{+}q_3)
\end{aligned}""",
    'C_compatta': r"""
C(q,\dot q)\,\dot q=\begin{bmatrix}
-l_1k_2\sin q_2\,(2\dot q_1\dot q_2+\dot q_2^2) - m_3\dot q_3^2\,\big(l_1\psi'_{13}+l_2\psi'_{123}\big)\\
-r\,\big(k_1\cos q_1\,\dot q_1^2 + k_2\cos(q_1{+}q_2)\,(\dot q_1{+}\dot q_2)^2 + m_3\,\psi_3\,\dot q_3^2\big)\\
l_1k_2\sin q_2\,\dot q_1^2 - m_3 l_2\,\psi'_{123}\,\dot q_3^2\\
-m_3\big(l_1\psi'_{13}\,\dot q_1^2 + l_2\psi'_{123}\,(\dot q_1{+}\dot q_2)^2\big)
\end{bmatrix}""",
    'G_compatta': r"""
G(q)=g\begin{bmatrix}
k_1\cos q_1 + k_2\cos(q_1{+}q_2)\\ 0\\ k_2\cos(q_1{+}q_2)\\ -m_3\,\psi(q_3)
\end{bmatrix}""",
    'M_num': r"""
M(q_0)=\begin{bmatrix}
0.1062 & -0.0353 & 0.0502 & 0.0029\\
-0.0353 & 0.0206 & -0.0167 & -0.0009\\
0.0502 & -0.0167 & 0.0502 & -0.0057\\
0.0029 & -0.0009 & -0.0057 & 0.0104
\end{bmatrix}\ \mathrm{kg\,m^2},\qquad
G(q_0)=\begin{bmatrix}-0.307\\0\\2.732\\-0.768\end{bmatrix}\mathrm{N\,m}""",
    'proprieta_M': r"""
\begin{aligned}
&M = M^{T} \succ 0:\quad \lambda(M) = \{0.0070,\ 0.0082,\ 0.0245,\ 0.1476\}\\
&\dot M - 2C \ \text{antisimmetrica}\ \Rightarrow\ \dot q^{T}(\dot M - 2C)\dot q = 0\quad\text{(passivit\`a)}\\
&G_2 = 0:\ q_w \ \text{ciclica (il suolo \`e piano e omogeneo)}
\end{aligned}""",
    'B_matrice': r"""
B=\begin{bmatrix}1&0&1\\1&0&0\\0&1&1\\0&0&1\end{bmatrix},\quad
\tau=\begin{bmatrix}\tau_w\\ \tau_k\\ \tau_h\end{bmatrix},\qquad
B^{T}=\frac{\partial}{\partial q}\begin{bmatrix}\varphi_w\\ \varphi_k\\ \varphi_h\end{bmatrix}
=\frac{\partial}{\partial q}\begin{bmatrix}q_w+q_1\\ q_2\\ q_3+q_1+q_2\end{bmatrix}""",
    'B_check': r"""
\delta q_{\text{rigido}} = [\,1,\,-1,\,0,\,-1\,]^{T}\delta:\qquad
B^{T}\delta q_{\text{rigido}} = \begin{bmatrix}0\\0\\0\end{bmatrix},\qquad
B_{\text{init.m}}^{T}\,\delta q_{\text{rigido}} = \begin{bmatrix}{\color{acc}-2}\\0\\{\color{acc}-2}\end{bmatrix}""",
    'schur': r"""
\begin{aligned}
&\begin{bmatrix}M_{uu} & M_{ua}\\ M_{au} & M_{aa}\end{bmatrix}\begin{bmatrix}\ddot q_u\\ \ddot q_a\end{bmatrix}
+\begin{bmatrix}h_u\\ h_a\end{bmatrix}=\begin{bmatrix}B_u\\ B_a\end{bmatrix}\tau,\qquad q_u = q_1,\ \ q_a=[q_w,q_2,q_3]\\
&D = B_a - M_{au}M_{uu}^{-1}B_u,\qquad \bar M = M_{aa}-M_{au}M_{uu}^{-1}M_{ua},\qquad
\bar h = h_a - M_{au}M_{uu}^{-1}h_u\\
&\tau = D^{-1}\big(\bar M\,v + \bar h\big)\ \Rightarrow\ \ddot q_a = v,\qquad
\ddot q_u = M_{uu}^{-1}\big(B_u\tau - M_{ua}v - h_u\big)
\end{aligned}""",
    'vlwip_nl': r"""
\begin{bmatrix} m_b{+}2m_w{+}\frac{2I_w}{r^2} & m_b l\cos\theta & 0\\ m_b l\cos\theta & m_b l^2{+}I_y & 0\\ 0&0&M_{33}\end{bmatrix}
\begin{bmatrix}\ddot s\\ \ddot\theta\\ \ddot\varphi\end{bmatrix}
+\begin{bmatrix}-m_b l\dot\theta^2\sin\theta\\ -m_b g l\sin\theta\\ 0\end{bmatrix}
=\begin{bmatrix}\frac1r & \frac1r\\ -1 & -1\\ -\frac{d}{2r} & \frac{d}{2r}\end{bmatrix}\begin{bmatrix}\tau_l\\ \tau_r\end{bmatrix}""",
    'vlwip_lin': r"""
\begin{aligned}
\ddot s &= a_1\,\theta + b_1\,(\tau_l{+}\tau_r), \qquad \ddot\theta = a_2\,\theta + b_2\,(\tau_l{+}\tau_r),\qquad
\ddot\varphi = b_3\,(\tau_r{-}\tau_l)\\
a_1 &= -\frac{g\,l^2m_b^2r^2}{\Delta},\quad a_2=\frac{g\,l\,m_b\big(2I_w{+}(m_b{+}2m_w)r^2\big)}{\Delta},\quad
b_1 = \frac{r\big(I_y{+}l\,m_b(l{+}r)\big)}{\Delta},\quad b_2 = -\frac{2I_w{+}r\big(l\,m_b{+}(m_b{+}2m_w)r\big)}{\Delta}
\end{aligned}""",
    'fase_non_minima': r"""
\frac{S(\sigma)}{U(\sigma)}=\frac{b_1\sigma^2+(a_1b_2-a_2b_1)}{\sigma^2(\sigma^2-a_2)}
\quad\Rightarrow\quad \text{poli } \pm\sqrt{a_2}=\pm 10.29,\quad \text{zeri } \pm 6.23\ \mathrm{rad/s}""",
    # ------------------------------------------------------------------ 2. C-space
    'dof': r"""
n = \underbrace{6}_{\text{torso}} + \underbrace{2}_{\text{anche}} + \underbrace{2}_{\text{ginocchia}} + \underbrace{2}_{\text{ruote}} = 12,
\qquad q=[x,y,z,\phi,\theta,\psi,\,q_{hl},q_{hr},\,q_{kl},q_{kr},\,q_{wl},q_{wr}]^{T}""",
    'cspace': r"""
\mathcal{C} = SE(3)\times\mathbb{R}^4\times T^2 \;\cong\; \mathbb{R}^3\times SO(3)\times[q^-,q^+]^4\times S^1\times S^1""",
    'pfaff': r"""
\begin{aligned}
&A^{T}(q)\,\dot q = 0:\qquad \dot x\sin\psi-\dot y\cos\psi = 0 \quad\text{(niente slittamento laterale)}\\
&\dot x\cos\psi+\dot y\sin\psi = \tfrac{r}{2}(\omega_l+\omega_r),\qquad \dot\psi = \tfrac{r}{d}(\omega_r-\omega_l)\\
&M(q)\,\ddot q + h(q,\dot q) = B(q)\,\tau + A(q)\,\lambda
\end{aligned}""",
    'sottoattuazione': r"""
m=\dim\tau = 6 \;<\; n = 12 \qquad\Rightarrow\qquad \operatorname{rank}B(q)=6<n""",
    # ------------------------------------------------------------------ 3. PID con ZMP
    'lipm': r"""
\ddot c = \omega^2\,(c-p),\qquad \omega^2=\frac{g}{h},\qquad
\xi = c + \frac{\dot c}{\omega},\qquad \dot\xi = \omega\,(\xi-p)""",
    'anello_A': r"""
\begin{aligned}
s_{des} &= s_{ff} - \rho\Big[\frac{\dot e_c}{\omega} + \frac{k_\xi}{\omega}\,e_\xi + \frac{k_i}{\omega}\int e_\xi\,\dd t\Big],
\qquad |s_{des}|\le \frac{a_{max}}{\omega^2}\\
\tau_c &= \tau_{ff} + K_s\,(s - s_{des}) + K_{sd}\,(\dot s-\dot s_{ff}),\qquad
s_{ff} = \lambda_s\,\ddot c_{ref},\ \ \tau_{ff}=\lambda_\tau\,\ddot c_{ref}
\end{aligned}""",
    'anteprima': r"""
c_{ref}(t) = \int_{-\infty}^{+\infty}\frac{\omega}{2}\,e^{-\omega|u|}\,p_{ref}(t+u)\,\dd u
\quad\Rightarrow\quad \ddot c_{ref} = \omega^2\big(c_{ref}-p_{ref}\big)""",
    'imbardata_pid': r"""
\tau_d = K_\psi e_\psi + K_\omega(\dot\psi_{ref}-\dot\psi) + f_c\tanh\!\Big(\frac{\dot\psi_{ref}}{\varepsilon}\Big) + f_v\,\dot\psi_{ref} + K_I\!\int e_\psi\,\dd t,
\qquad \psi_{cmd} = \psi_{ref} - \arctan(K_{lat}e_\perp)\,\min\!\Big(1,\frac{|v_{ref}|}{0.1}\Big)""",
    'gambe_pd': r"""
\tau_{leg} = J^{T}(q)\Big[K_p\,(p_{des}-p) - K_d\,J\dot q + F_{ff}\Big] + b\,\dot q,
\qquad F_{ff} = -\tfrac12 m_b g\,\hat z_b""",
    # ------------------------------------------------------------------ 4. MPC + TV-LQR
    'lqr': r"""
\begin{aligned}
&J = \int_0^\infty\big(\tilde X^{T}Q\tilde X + U^{T}RU\big)\,\dd t,\qquad U = -K(l)\,\tilde X\\
&A^{T}P + PA - PBR^{-1}B^{T}P + Q = 0,\qquad K = R^{-1}B^{T}P
\end{aligned}""",
    'pesi_lqr': r"""
Q = \operatorname{diag}(30,\,400,\,80,\,15,\,6,\,2),\qquad
R = T^{T}\operatorname{diag}(2,\,100)\,T,\quad T=\begin{bmatrix}\tfrac12&\tfrac12\\-\tfrac12&\tfrac12\end{bmatrix}
\ \Rightarrow\ U^{T}RU = 2\tau_c^2+100\,\tau_d^2""",
    'K_lqr': r"""
K(l_0)=\begin{bmatrix}-3.87&-20.28&-0.89&-5.64&-2.78&-0.21\\-3.87&-20.28&+0.89&-5.64&-2.78&+0.21\end{bmatrix}""",
    'mpc_modelli': r"""
\begin{aligned}
\begin{bmatrix}s\\ \dot s\end{bmatrix}_{k+1} &= \begin{bmatrix}1&\Delta T\\0&1\end{bmatrix}\begin{bmatrix}s\\ \dot s\end{bmatrix}_k
+ \begin{bmatrix}\Delta T^2/2\\ \Delta T\end{bmatrix}\frac{g+\ddot z_{ref}}{h}\,\Delta s_k\\
\begin{bmatrix}z\\ \dot z\end{bmatrix}_{k+1} &= \begin{bmatrix}1&\Delta T\\0&1\end{bmatrix}\begin{bmatrix}z\\ \dot z\end{bmatrix}_k
+ \begin{bmatrix}\Delta T^2/2\\ \Delta T\end{bmatrix}\Big(\frac{F_{z,k}}{m_b}-g\Big)
\end{aligned}""",
    'mpc_qp': r"""
\min_{U}\ \tfrac12 U^{T}HU + g^{T}U\quad \text{s.t.}\ \ lo\le U\le hi,\qquad
H = \Gamma^{T}\bar S\,\Gamma + W I,\quad g = \Gamma^{T}\bar S\,(\Phi x_0 + C - x_{ref})""",
    'mpc_vincoli': r"""
|\Delta s| \le \min\!\Big(\mu z,\ \sqrt{L_{max}^2-z_b^2},\ 0.03\Big),\qquad
0.3\,m_b g \le F_z \le \{3;\,6\}\,m_b g,\qquad
\theta_{ref}=\operatorname{atan2}(\Delta s, z_{ref})""",
    'xref': r"""
X_{ref}=\big[\,s_{plan}-\Delta s,\ \operatorname{atan2}(\Delta s,z_{ref}),\ \varphi_{ref},\ \dot s_{plan},\ 0,\ \dot\varphi_{ref}\,\big]^{T}""",
    'vmc': r"""
\tau_{leg} = J^{T}\big[K_p(p_d-p_f) + K_d(v_d-v_f) + F_{ff}\big] + b_{leg}\,\dot q,\qquad
F_{ff}=-\tfrac12F_z\,\hat z_b,\quad K_{p,z}=0""",
    'kalman': r"""
\begin{aligned}
&x_{k|k-1} = F x_{k-1} + \Gamma a_{w,k},\quad P_{k|k-1} = FP_{k-1}F^{T}+Q\\
&K_k = P_{k|k-1}\big(P_{k|k-1}+R\big)^{-1},\quad x_{k} = x_{k|k-1}+K_k\big(y_k-x_{k|k-1}\big)
\end{aligned}""",
}


def render(name, body):
    tmp = tempfile.mkdtemp()
    tex = os.path.join(tmp, 'eq.tex')
    with open(tex, 'w') as f:
        f.write(PREAMBLE + body.strip() + '\n$\n\\end{document}\n')
    res = subprocess.run(['pdflatex', '-interaction=nonstopmode', '-halt-on-error', 'eq.tex'],
                         cwd=tmp, capture_output=True, text=True)
    if res.returncode != 0:
        raise RuntimeError(f'{name}: LaTeX fallito\n' + res.stdout[-2000:])
    info = subprocess.run(['pdfinfo', 'eq.pdf'], cwd=tmp, capture_output=True, text=True).stdout
    w_pt, h_pt = [float(x) for x in info.split('Page size:')[1].split('pts')[0].split('x')]
    subprocess.run(['pdftocairo', '-png', '-transp', '-singlefile', '-r', str(DPI), 'eq.pdf',
                    os.path.join(OUT, name)], cwd=tmp, check=True)
    shutil.rmtree(tmp)
    return {'w': w_pt / 72.0, 'h': h_pt / 72.0}


def main():
    os.makedirs(OUT, exist_ok=True)
    manifest = {name: render(name, body) for name, body in EQ.items()}
    with open(os.path.join(OUT, 'manifest.json'), 'w') as f:
        json.dump(manifest, f, indent=1)
    for k, v in manifest.items():
        print(f'{k:18s} {v["w"]:.2f} x {v["h"]:.2f} in')


if __name__ == '__main__':
    main()
