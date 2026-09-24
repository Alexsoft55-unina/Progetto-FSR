# Tecniche di controllo di RoTino

Documento tecnico sulle tre leggi di controllo del workspace `~/rotino_ws`: **PID** (`rotino_pid`), **MPC + TV-LQR + VMC** (`rotino_mpc`) e **cascata sliding mode** (`rotino_smc`). Tutte e tre agiscono sullo stesso URDF, con la stessa attuazione in coppia a 500 Hz e la stessa libreria di modello (`rotino_description`). Il documento descrive il modello dinamico su cui poggiano, la struttura di ogni anello, l'origine numerica dei guadagni e delle costanti, e chiude con un'analisi del comportamento ai disturbi di MPC e SMC.

Scritto il 24/09/2026 sul codice presente in quel momento nel workspace.

## Come leggere le affermazioni

Ogni risultato quantitativo porta una di queste etichette:

| Etichetta | Significato |
|---|---|
| **[D]** | Derivato analiticamente nel testo, a partire dal modello. |
| **[C]** | Calcolato eseguendo il codice del workspace (`WBRModel`, `lqr_gain`, `UpperBodyMPC`, `AxisKalman`) con i parametri reali dell'URDF. |
| **[S-R]** | Simulato su modello ridotto: VL-WIP sagittale non lineare (eq. 4-5 di Cui et al.), integrazione RK4 a 2 kHz, controllo a 500 Hz con gli stessi filtri e saturazioni dei nodi. |
| **[S-G]** | Misurato in Gazebo per questo documento, con `rotino_benchmark campaign` e con la registrazione di `/rotino/wbr_state`. |
| **[E]** | Empirico riportato: misura dichiarata nei commenti del codice, in `docs/Storico_lavoro.md` o in `diagnostics/06_diagnosi.txt`, non ripetuta qui. |

Gli script che producono i numeri [C], [S-R] e [S-G] sono in `docs/verifiche/` (Appendice C).

Riferimento bibliografico principale: Z. Cui, Y. Xin, S. Liu, X. Rong, Y. Li, *Modeling and Control of a Wheeled Biped Robot*, Micromachines 2022, 13, 747 (nel seguito "il paper"). I numeri di equazione citati sono quelli del paper.

---

## 0. Convenzioni

**Terne.** REP 103: $x$ in avanti, $y$ a sinistra, $z$ in alto. Tutti i giunti (anca, ginocchio, ruota) ruotano attorno a $+y$. La ruota sinistra sta a $+y$.

**Coordinate generalizzate del VL-WIP.** $\phi = [s,\ \theta,\ \varphi]^T$:
- $s$: spostamento dell'asse ruote lungo la direzione di marcia [m];
- $\theta$: inclinazione del pendolo equivalente, cioè del baricentro del corpo superiore rispetto all'asse, positiva quando il baricentro è **davanti** all'asse [rad];
- $\varphi$: imbardata [rad].

**Segno delle coppie.** Una coppia positiva sul giunto ruota (asse $+y$) fa ruotare la ruota in avanti: il centro ruota avanza con velocità $\omega r$, perché $\omega\hat y \times r\hat z = \omega r\,\hat x$. La reazione sul corpo è una coppia $-\tau$ attorno a $+y$. Una rotazione positiva attorno a $+y$ porta $\hat z$ verso $+\hat x$, cioè inclina il baricentro in avanti. Ne segue che una coppia positiva alle ruote **accelera l'asse in avanti e fa ruotare il pendolo all'indietro**: nel modello linearizzato $b_1 > 0$ e $b_2 < 0$. MPC e SMC inviano le coppie senza inversioni di segno. Il PID lavora in unità normalizzate con un segno motore `WHEEL_MOTOR_SIGN = -1` e un angolo di inclinazione definito con segno opposto; la conversione è nel §3.2.

**Modo comune e differenziale.** Con $\tau_l, \tau_r$ le coppie delle due ruote:
$$\tau_c = \tfrac12(\tau_l+\tau_r), \qquad \tau_d = \tfrac12(\tau_r-\tau_l), \qquad \tau_l = \tau_c-\tau_d,\ \ \tau_r = \tau_c+\tau_d .$$
Il modo comune agisce sul piano sagittale ($s$, $\theta$), il differenziale sull'imbardata.

---

## 1. Il modello dinamico

### 1.1 Parametri fisici

Tutti i parametri vengono letti dall'URDF (`src/rotino_description/urdf/rotino.urdf.xacro`) da `WBRModel` (`src/rotino_description/rotino_description/model.py:76`). Nessun valore è copiato a mano nei controllori, con l'unica eccezione del PID (§3.1). **[C]**

| Simbolo | Valore | Origine |
|---|---|---|
| $m_w$ | 0,38 kg | ruota 0,32 + mozzo 0,06 |
| $I_w$ | $2{,}595\cdot10^{-3}$ kg·m² | $i_{yy}$ della ruota, include l'armatura MuJoCo di 0,002 |
| $r$ | 0,06 m | raggio del cilindro di collisione |
| $d$ | 0,294 m | distanza fra i centri ruota |
| $m_b$ | 3,53 kg | massa totale (4,29 kg) meno le due ruote |
| $m_3$ | 2,85 kg | torso: `base_link` 2,50 + zavorra 0,35 |
| $m_2,\ m_1$ | 0,18 kg, 0,16 kg | coscia, stinco (per gamba) |
| $l_1 = l_2$ | 0,1300 m | $\sqrt{2}\cdot 0{,}0919$: segmenti inclinati di 45° |
| $L_{max}$ | 0,2599 m | $l_1+l_2$ |
| $\mu$ | 2,2 | `mu1` delle ruote (DART combina con il minimo; il suolo ha 2,2) |
| $\tau_{w,max}$ | 18 N·m | limite URDF della ruota (motore MuJoCo, gear 18) |
| $\tau_{leg,max}$ | 60 N·m | limite URDF di anca e ginocchio |
| $b_{leg}$ | 0,8 N·m·s/rad | smorzamento viscoso URDF di anca e ginocchio |

Tre aspetti del robot pesano sul controllo:

1. **Rapporto di massa.** Le ruote sono l'18% della massa totale e, soprattutto, $2I_w/r^2 = 1{,}44$ kg: l'inerzia rotazionale delle ruote, riportata alla traslazione, vale il 41% di $m_b$. Per questo il termine in $I_w$ nei coefficienti (eq. 14) non è trascurabile.
2. **Baricentro del torso avanzato.** La zavorra è montata a $(+0{,}045;\ 0;\ -0{,}045)$ m rispetto alla `base_link` e il box del torso ha il baricentro a $x = +0{,}025$ m. Il baricentro del torso sta a $(+0{,}0275;\ -0{,}0055)$ m dall'anca.
3. **Inerzia del corpo superiore piccola rispetto alla Tabella 1 del paper.** Il paper assume $I_y = \tfrac13 m_b l^2$ (asta omogenea incernierata all'estremo). Per RoTino l'inerzia reale attorno al baricentro, calcolata dai tensori dell'URDF, è $I_y = 0{,}0174$ kg·m² nella posa nominale, contro $\tfrac13 m_b l^2 = 0{,}0311$ kg·m². Il valore della Tabella 1 sarebbe 1,8 volte troppo grande. MPC e SMC usano il valore reale (§1.5).

### 1.2 Cinematica sagittale della gamba

Ogni gamba è una catena planare di due link con giunti rotoidali attorno a $y$. Nella posa di riferimento ($q_{hip} = q_{knee} = 0$) il ginocchio sta a $(-a, -a)$ dall'anca e la ruota a $(+a, -a)$ dal ginocchio, con $a = 0{,}0919$ m. La ruota cade quindi esattamente sotto l'anca, $0{,}1838$ m più in basso.

`leg_fk` (`model.py:211`) calcola la posizione del centro ruota nella terna `base_link` e lo Jacobiano rispetto a $(q_{hip}, q_{knee})$. Con $R_y(q)$ la rotazione attorno a $y$ proiettata sul piano $(x,z)$:
$$p_f(q) = p_{hip} + R_y(q_{hip})\,p_{k} + R_y(q_{hip}+q_{knee})\,p_{w}, \qquad J = \frac{\partial p_f}{\partial q} .$$
Nella posa nominale **[C]**:
$$J_0 = \begin{bmatrix} -0{,}1838 & -0{,}0919 \\ 0 & -0{,}0919\end{bmatrix}, \qquad \det J_0 = 0{,}01689\ \text{m}^2, \qquad J_0^{-T} = \begin{bmatrix} -5{,}44 & 0 \\ 5{,}44 & -10{,}88\end{bmatrix}\ \text{m}^{-1}.$$
$J_0^{-T}$ converte coppie di giunto in forze al centro ruota: 1 N·m al ginocchio corrisponde a circa 10,9 N verticali. Il numero serve per dimensionare il termine robusto dello SMC delle gambe (§5.5).

**Vincolo $q_{hip} = -q_{knee}/2$.** Con questo vincolo la ruota resta sulla verticale dell'anca e il torso resta orizzontale. La tabella dell'altezza usata da MPC e SMC per schedulare i guadagni e per il salto campiona 25 pose con $q_{hip} \in [-0{,}45;\ 0{,}45]$ e $q_{knee} = -2q_{hip}$ (`controller.py:169-175` in entrambi i package).

### 1.3 Centroide equivalente (eq. 2-3)

Il corpo superiore (torso, zavorra, cosce, stinchi) viene ridotto a un punto materiale di massa $m_b$ nel suo baricentro. Nella terna dell'asse:
$$ {}^wP_C = \frac{\sum_i m_i\,{}^wP_{Ci}(q)}{\sum_i m_i} = [S_C,\ Z_C]^T, \qquad l = \sqrt{S_C^2+Z_C^2}, \qquad \theta = \operatorname{atan2}(S_C, Z_C).$$
`equivalent_centroid` (`model.py:151`) calcola anche l'inerzia $I_y$ attorno al baricentro, sommando i tensori dei link ruotati in terna mondo e i termini di trasporto di Steiner. **[C]**

| $q_{hip}$ | $q_{knee}$ | $z_b$ [m] | $S_C$ [m] | $Z_C$ [m] | $l$ [m] | $\theta$ a torso orizzontale | $I_y$ [kg·m²] |
|---|---|---|---|---|---|---|---|
| −0,45 | +0,90 | 0,2454 | +0,0180 | 0,2180 | 0,2188 | +4,73° | 0,0201 |
| −0,30 | +0,60 | 0,2299 | +0,0163 | 0,2040 | 0,2046 | +4,58° | 0,0194 |
| −0,15 | +0,30 | 0,2092 | +0,0147 | 0,1852 | 0,1858 | +4,55° | 0,0184 |
| **0** | **0** | **0,1838** | **+0,0133** | **0,1622** | **0,1627** | **+4,69°** | **0,0174** |
| +0,15 | −0,30 | 0,1543 | +0,0121 | 0,1354 | 0,1359 | +5,10° | 0,0163 |
| +0,30 | −0,60 | 0,1213 | +0,0111 | 0,1055 | 0,1061 | +6,00° | 0,0153 |
| +0,45 | −0,90 | 0,0856 | +0,0103 | 0,0731 | 0,0738 | +8,06° | 0,0145 |

$z_b$ è la distanza verticale anca-asse. La colonna $\theta$ è l'inclinazione **a torso orizzontale**: con il robot in quella posa il baricentro sta 1,0-1,8 cm davanti all'asse. Il robot in equilibrio non può quindi avere il torso orizzontale. In equilibrio $\theta = 0$, cioè il torso è ruotato all'indietro di circa 4,7° nella posa nominale. Questo spiega il `PITCH_OFFSET` del PID (§3.2).

**Equivalenza fra baricentro totale e baricentro del corpo superiore.** Il PID misura l'inclinazione del baricentro dell'**intero** robot, MPC e SMC quella del **solo corpo superiore**. Le due coincidono. I baricentri delle ruote giacciono sull'asse, quindi rispetto al punto medio dell'asse $m_{tot}\,\Delta x_{tot} = m_b S_C$ e $m_{tot}\,\Delta z_{tot} = m_b Z_C$. Il rapporto $\Delta x/\Delta z$ e quindi l'angolo sono identici. **[D]**

### 1.4 Il VL-WIP non lineare (eq. 4-5)

Il robot viene ridotto a un pendolo inverso su ruote a lunghezza variabile (VL-WIP). Il pendolo ha massa $m_b$, lunghezza $l$ e inerzia $I_y$ attorno al proprio baricentro; lo portano due ruote di massa $m_w$ e inerzia $I_w$. Dalle equazioni di Eulero-Lagrange, a $l$ congelato:
$$M(\phi)\ddot\phi + C(\phi,\dot\phi) = B\,\tau_w, \qquad \tau_w = [\tau_l,\ \tau_r]^T$$
$$M = \begin{bmatrix} m_b+2m_w+\frac{2I_w}{r^2} & m_b l\cos\theta & 0\\ m_b l\cos\theta & m_b l^2 + I_y & 0 \\ 0 & 0 & \frac{d^2 m_w}{2}+\frac{d^2 I_w}{2r^2}+I_z\end{bmatrix},\quad C = \begin{bmatrix} -m_b l\dot\theta^2\sin\theta \\ -m_b g l \sin\theta \\ 0\end{bmatrix},\quad B = \begin{bmatrix} r^{-1} & r^{-1}\\ -1 & -1 \\ \frac{d}{2r} & -\frac{d}{2r}\end{bmatrix}.$$

Significato dei termini:
- $M_{11}$ è la massa traslante equivalente: corpo, due ruote e l'inerzia di rotolamento delle ruote $2I_w/r^2$.
- $M_{12} = m_b l\cos\theta$ è l'accoppiamento inerziale: accelerare l'asse fa ruotare il pendolo e viceversa. È l'origine della fase non minima (§1.6).
- $C_2 = -m_b g l\sin\theta$ è la coppia di gravità che rende instabile l'equilibrio.
- La riga 2 di $B$ vale $-1$: la coppia motore agisce sul pendolo con segno opposto a quello con cui fa girare la ruota (reazione).
- L'imbardata è disaccoppiata: $M$ è diagonale a blocchi e $C_3 = 0$.

Nel paper la terza riga di $B$ è $\left[\frac{d}{2r},\ -\frac{d}{2r}\right]$ e porta a $B(l)$ con $[b_3,\ -b_3]$. Nel workspace la ruota sinistra è a $+y$, quindi una coppia in avanti sulla sinistra fa girare il robot in senso orario: `vlwip_matrices` usa $[-b_3,\ b_3]$ (`model.py:207-208`). È un cambio di convenzione, non di fisica.

Il modello trascura: la dinamica di $l$ ($\dot l$, $\ddot l$ e i relativi termini di Coriolis), la rotazione del torso rispetto al pendolo equivalente, la cedevolezza delle gambe, lo slittamento e il distacco delle ruote.

### 1.5 Linearizzazione (eq. 13-14)

Attorno a $\theta = 0$, $\dot\theta = 0$, con $\sin\theta\approx\theta$, $\cos\theta\approx 1$ e $\dot\theta^2\approx 0$, e con $X = [s,\ \theta,\ \varphi,\ \dot s,\ \dot\theta,\ \dot\varphi]^T$ e $U = [\tau_l,\ \tau_r]^T$:
$$\dot X = A(l)X + B(l)U,\qquad \ddot s = a_1\theta + b_1(\tau_l+\tau_r),\quad \ddot\theta = a_2\theta + b_2(\tau_l+\tau_r),\quad \ddot\varphi = b_3(\tau_r-\tau_l)$$
con, posto $\Delta = 2I_w(I_y+m_bl^2) + \big(2l^2m_bm_w + I_y(m_b+2m_w)\big)r^2$:
$$a_1 = -\frac{g\,l^2m_b^2r^2}{\Delta},\quad a_2 = \frac{g\,l\,m_b\big(2I_w+(m_b+2m_w)r^2\big)}{\Delta},\quad b_1 = \frac{r\big(I_y+l\,m_b(l+r)\big)}{\Delta},\quad b_2 = -\frac{2I_w+r\big(l\,m_b+(m_b+2m_w)r\big)}{\Delta}$$
$$b_3 = \frac{d\,r}{2I_zr^2+d^2(I_w+m_wr^2)} .$$

**Verifica.** Le espressioni di `vlwip_coefficients` (`model.py:180-195`) coincidono con l'inversione simbolica di $M$ linearizzata applicata a $[u/r,\ -u + m_bgl\theta]^T$, con $u = \tau_l+\tau_r$: la differenza, calcolata con SymPy, è identicamente zero. $b_1$ e $b_2$ moltiplicano **ciascuna** coppia di ruota, cioè la loro somma. **[D]+[C]**

Coefficienti lungo la griglia di pose, con $I_y$ reale **[C]**:

| $l$ [m] | $a_1$ [m/s²] | $a_2$ [s⁻²] | $b_1$ [m/(s²·N·m)] | $b_2$ [1/(s²·N·m)] | $\sqrt{a_2}$ [rad/s] |
|---|---|---|---|---|---|
| 0,2188 | −12,02 | 89,17 | 8,055 | −38,20 | 9,44 |
| 0,2046 | −11,73 | 93,07 | 8,040 | −40,72 | 9,65 |
| 0,1858 | −11,29 | 98,64 | 8,007 | −44,57 | 9,93 |
| **0,1627** | **−10,60** | **105,82** | **7,933** | **−50,15** | **10,29** |
| 0,1359 | −9,54 | 113,94 | 7,762 | −57,98 | 10,67 |
| 0,1061 | −7,86 | 120,35 | 7,379 | −68,44 | 10,97 |
| 0,0738 | −5,32 | 116,94 | 6,564 | −80,40 | 10,81 |

$b_3 = 34{,}88$ rad/(s²·N·m) con $I_z = 0{,}0227$ kg·m² della posa nominale, tenuto costante.

Con l'$I_y$ della Tabella 1 ($\tfrac13 m_b l^2$), in posa nominale si otterrebbe $a_2 = 84{,}0$ e $b_2 = -39{,}8$: 20% di instabilità in meno e 21% di autorità in meno rispetto al robot reale. Un super-twisting che inverte $b_2$ (§5.3) sbaglierebbe la coppia di quel fattore.

### 1.6 Proprietà strutturali

**Instabilità.** Il sottosistema $(\theta, \dot\theta)$ ad anello aperto ha i poli $\pm\sqrt{a_2}$. In posa nominale il polo instabile è a $+10{,}29$ rad/s: costante di tempo di divergenza 97 ms. Accorciando le gambe il polo sale fino a circa 11 rad/s. **[C]**

**Fase non minima.** Dalla coppia totale $u$ allo spostamento dell'asse, trasformando con Laplace le due equazioni sagittali:
$$\frac{S(\sigma)}{U(\sigma)} = \frac{b_1\sigma^2 + (a_1b_2 - a_2b_1)}{\sigma^2(\sigma^2 - a_2)} .$$
Lo zero sta in $\sigma^2 = (a_2b_1 - a_1b_2)/b_1$. In posa nominale $(105{,}82\cdot7{,}933 - 10{,}60\cdot50{,}15)/7{,}933 = 38{,}8$, quindi **zeri reali in $\pm 6{,}23$ rad/s**, di cui uno a parte reale positiva (da 5,67 a 7,20 rad/s lungo la griglia). **[D]+[C]**

In termini fisici: per accelerare in avanti a regime il baricentro deve stare davanti all'asse. Per portarcelo le ruote devono prima arretrare. Una coppia positiva produce subito $\ddot s = b_1u > 0$, ma ruota il pendolo all'indietro ($b_2 < 0$); la gravità prende poi il sopravvento e il moto netto si inverte. Nessun controllore causale può seguire un gradino di velocità senza questa sottoelongazione iniziale.

**Guadagno statico inclinazione → accelerazione.** Se il pendolo è tenuto a inclinazione costante ($\ddot\theta = 0$), la coppia necessaria è $u = -a_2\theta/b_2$ e l'asse accelera con
$$\ddot s = \Big(a_1 - \frac{b_1a_2}{b_2}\Big)\theta \equiv g_{st}\,\theta,\qquad g_{st} = 6{,}14\ \text{m/(s}^2\text{·rad)}\ \text{in posa nominale}.$$
**[D]+[C]** È il meccanismo su cui si reggono l'anello esterno dello SMC (§5.2) e l'MPC (§4.5): **l'inclinazione è l'ingresso effettivo della traslazione**. Il suo limite superiore fissa la massima accelerazione ottenibile. Per inclinazioni costanti di 10,48° e 12,03°, che sono i limiti di MPC e SMC, $g_{st}\tan\theta$ vale rispettivamente 1,13 e 1,31 m/s². La formula usa la tangente perché così è definito $\theta_{ref}$ nell'MPC; per angoli piccoli coincide con $g_{st}\theta$.

**Disaccoppiamento.** $A$ e $B$ sono a blocchi: sagittale $(s, \theta, \dot s, \dot\theta)$ su $\tau_l+\tau_r$, imbardata $(\varphi, \dot\varphi)$ su $\tau_r-\tau_l$. Nel modello lineare il bilanciamento e la sterzata sono indipendenti.

### 1.7 Il corpo superiore come massa concentrata (eq. 6-9)

Per pianificare lo spostamento del baricentro il paper usa un secondo modello. Il corpo superiore è una massa $m_b$ ad altezza $h$, spostata di $\Delta s$ in orizzontale rispetto al punto di contatto. La condizione di non ribaltamento richiede che la risultante di gravità e forza d'inerzia passi per il punto di appoggio (eq. 6):
$$\frac{F_s}{\Delta s} = \frac{F_z}{h},\qquad F_s = m\ddot s,\quad F_z = m(g+\ddot z)\quad\Longrightarrow\quad \ddot s = \frac{g+\ddot z}{h}\,\Delta s \quad\text{(eq. 7)}$$
In verticale la massa è spinta dalla forza delle gambe (eq. 8):
$$\ddot z = \frac{F_z}{m_b} - g .$$
Lo stato $x = [s,\ \dot s,\ z,\ \dot z,\ -g]$ con ingresso $u = [\Delta s,\ F_z]$ dà l'eq. 9, che `upper_body_matrices` (`model.py:225`) riproduce. Per $\ddot z$ noto, l'eq. 7 è un pendolo inverso lineare con ingresso la posizione del centro di pressione relativa al baricentro. Il tempo caratteristico è $\sqrt{h/g} = 0{,}129$ s a $h = 0{,}162$ m. **[D]**

I due modelli si completano. Il VL-WIP descrive come le ruote tengono in equilibrio il pendolo; il modello a massa concentrata descrive dove conviene mettere il baricentro rispetto alle ruote, e con quale forza verticale, per muovere il corpo. $\Delta s$ del secondo modello e $\theta$ del primo sono legati da $\theta = \operatorname{atan2}(\Delta s, z)$ (§4.3).

### 1.8 Tempo discreto, filtri e ritardi

- **Frequenze.** Fisica a 2 kHz (`max_step_size` 0,5 ms), `controller_manager` a 500 Hz, odometria a 500 Hz. I tre nodi eseguono un passo per ogni messaggio di `/joint_states`: $\Delta t = 2$ ms, misurato costante **[E]**. L'MPC gira ogni cinque passi (100 Hz).
- **Filtri del primo ordine** $y_k = \alpha y_{k-1} + (1-\alpha)x_k$ a 500 Hz. La costante di tempo equivalente è $\tau = -\Delta t/\ln\alpha$: con $\alpha = 0{,}8$ (su $\dot\theta$ e $\dot s$ in MPC e SMC) $\tau = 8{,}96$ ms; con $\alpha = 0{,}9$ (su $\Delta s$ nell'MPC) $\tau = 19{,}0$ ms. **[D]** Alla frequenza del polo instabile (10,3 rad/s) il filtro su $\dot\theta$ introduce 5,3° di ritardo di fase. A 70 rad/s ne introduce 32°. Il secondo valore è rilevante per il ciclo limite dello SMC (§6.7).
- **$\dot\theta$** è ottenuta per differenza finita di $\theta$ (calcolato dalla cinematica e dall'IMU) e poi filtrata. Nessun controllore usa direttamente la velocità di beccheggio del giroscopio per $\dot\theta$: $\theta$ è l'angolo del baricentro rispetto all'asse, che cambia anche per il moto delle gambe.

---

## 2. Stima dello stato (MPC e SMC)

MPC e SMC usano lo stesso stimatore, riga per riga (`rotino_mpc/controller.py:504-574`, `rotino_smc/controller.py:583-650`). Il PID **non** lo usa: legge posa e velocità del torso dalla ground truth del simulatore (`/rotino/odom`) e ne deriva stati senza filtro. Nel confronto è una differenza di condizioni, non di legge di controllo, e va tenuta presente.

### 2.1 Grandezze misurate

- **IMU** (500 Hz): quaternione di orientazione $R$, velocità angolare $\omega_b$, forza specifica $f_b$. L'accelerazione in terna mondo è $a_w = Rf_b - [0,0,g]^T$.
- **Encoder** su anche, ginocchia e ruote: posizioni e velocità.
- **Sensori di contatto** delle ruote: forza di contatto con il suolo, usata solo per lo stato di appoggio.

La direzione di marcia è la proiezione orizzontale normalizzata della prima colonna di $R$; l'imbardata è $\varphi = \operatorname{atan2}(h_y, h_x)$.

### 2.2 Filtro di Kalman lineare (eq. 20-21)

Per ciascun asse mondo, indipendentemente, lo stato è $[p_b,\ v_b]$ del torso (origine della `base_link`). Il modello di processo è un doppio integratore guidato dall'accelerazione IMU:
$$\begin{bmatrix}p\\v\end{bmatrix}_{k+1} = \begin{bmatrix}1&\Delta t\\0&1\end{bmatrix}\begin{bmatrix}p\\v\end{bmatrix}_k + \begin{bmatrix}\Delta t^2/2\\ \Delta t\end{bmatrix}a_{w,k} + w_k,\qquad Q = q_a\,\Gamma\Gamma^T,\ \ \Gamma = [\Delta t^2/2,\ \Delta t]^T .$$
L'osservazione è la posizione e la velocità del torso ricostruite dall'odometria ruote e dalla cinematica delle gambe:
$$y = \begin{bmatrix}P_w + {}^wP_b\\ V_w + {}^wV_b\end{bmatrix},\qquad {}^wP_b = -R\,p_{axle}^{b},\qquad {}^wV_b = -\omega_w\times(R\,p_{axle}^b) - R\,\dot p_{axle}^b .$$
$p_{axle}^b$ è il punto medio dei due centri ruota nella terna `base_link`, dato da `leg_fk`, e $\dot p^b_{axle}$ la sua velocità $J\dot q$.

**Odometria.** La velocità assoluta di rotazione di ciascuna ruota attorno a $y$ è la somma delle velocità lungo la catena seriale, perché tutti gli assi sono paralleli:
$$\omega_{ruota} = \dot q_{ruota} + \omega_{pitch} + \dot q_{hip} + \dot q_{knee} .$$
Con puro rotolamento $V_w = \tfrac r2(\omega_l+\omega_r)\,\hat h$. $P_w$ è l'integrale di $V_w$ con quota fissata a $r$. Usare solo $\dot q_{ruota}$ attribuirebbe alla traslazione ogni oscillazione di beccheggio e ogni movimento delle gambe.

**Parametri e guadagno a regime.** $q_a = 0{,}5$ (m/s²)², $R = \operatorname{diag}(10^{-4}\ \text{m}^2,\ 10^{-3}\ \text{m}^2/\text{s}^2)$. Il guadagno a regime per passo è **[C]**
$$K_\infty = \begin{bmatrix}0{,}0070 & 0{,}0017\\ 0{,}0168 & 0{,}0434\end{bmatrix}.$$
Gli autovalori di $(I-K_\infty)F$ sono 0,9929 e 0,9567, cioè costanti di tempo di 280 ms e 45 ms. L'IMU domina sotto i 45 ms e l'odometria sopra i 280 ms. Nel mezzo le due fonti si fondono. Il filtro serve a eliminare il rumore di derivazione e i salti dell'odometria, non a correggere derive lente dell'IMU. Queste sono comunque limitate perché l'orientazione arriva già stimata.

**Gating sul contatto.** La correzione avviene solo in appoggio. In volo l'odometria non ha significato, $P_w$ viene riallineato alla stima ($P_w = P_b - {}^wP_b$) e il filtro procede in sola predizione. Il paper, invece, gonfia la covarianza di osservazione durante il salto (Sez. 4.3); l'implementazione la esclude del tutto.

**Differenze dal paper.** Il paper usa un solo filtro con $P_b, V_b\in\mathbb R^2$. Qui ci sono tre filtri scalari indipendenti, uno per asse mondo. L'equivalenza vale perché $F$, $Q$ e $R$ sono diagonali a blocchi per asse.

### 2.3 Grandezze derivate

- $s$: integrale della proiezione della velocità dell'asse $V_{axle} = V_b - {}^wV_b$ sulla direzione di marcia.
- $\dot s$: la stessa proiezione, filtrata con $\alpha = 0{,}8$.
- $\theta$, $l$: dal centroide equivalente (§1.3), usando $R$ per portare in terna mondo il vettore asse→baricentro.
- $\dot\theta$: differenza finita filtrata (§1.8).

L'MPC usa in più la velocità del **baricentro** $V_{com} = V_b + \omega_w\times(Rc_b)$ come stato del proprio modello orizzontale (`rotino_mpc/controller.py:570-572`). Lo SMC non la usa. Il perché è nel §5.2.

### 2.4 Stato di appoggio

Isteresi sulla forza normale totale (spegnimento sotto 1,5 N, accensione sopra 6 N) con conferma temporale (4 ms per la perdita, 10 ms per il ritorno) e scarto dei messaggi più vecchi di 6 ms. Il sensore di Gazebo pubblica solo mentre c'è contatto, quindi l'assenza di messaggi significa assenza di contatto. L'asimmetria dei tempi di conferma rileva in fretta il decollo e scarta i rimbalzi all'atterraggio.

**La forza di contatto in Gazebo non è misurata.** In tutte le prove eseguite per questo documento (MPC e SMC, 33.000 campioni) la forza normale totale pubblicata su `/rotino/debug` vale sempre esattamente 34,629 N **[S-G]**. È $2\cdot\tfrac12 m_bg$, il valore di ripiego che `_contact_cb` assegna quando un messaggio di contatto non contiene wrench (MPC riga 283-284, SMC riga 316-317). Non è il peso totale (42,1 N), che è quello che un sensore reale riporterebbe. Il sensore di contatto di Gazebo, con questa configurazione, segnala quindi la presenza del contatto ma non la sua intensità. Ne seguono due fatti:
- le soglie di 1,5 N e 6 N si riducono a "messaggio presente o assente";
- ogni grandezza calcolata dalla forza normale è di fatto costante in appoggio: in particolare il limite di aderenza dello SMC (§5.3) vale 1,83 N·m per ruota.

---

## 3. Controllore PID

`src/rotino_pid/rotino_pid/controller.py`. È il porting a ROS 2 di un controllore MuJoCo (`RoTino_Ctrl_BalJumpAchieve.py`) nato come bilanciamento con salto verticale. Le gambe erano servo di posizione dentro il simulatore; nel workspace sono state **portate a coppia** perché le tre leggi usino la stessa attuazione (`docs/Storico_lavoro.md` §2.4).

### 3.1 Struttura degli anelli

Il nome "PID" è storico. Nel codice non ci sono termini integrali: ci sono quattro gruppi di anelli proporzionali-derivativi coordinati da un supervisore a stati.

```
                        ┌───────────────────────────── supervisore del salto (7 stati) ─────────────────────────────┐
                        │  sceglie: guadagni ruote per fase, riferimenti di giunto, x_ref dopo l'atterraggio          │
                        └───────┬───────────────────────────────┬───────────────────────────────────────────────────┘
                                │                               │
   x_ref, ẋ_ref ──►  (A) retroazione di stato sagittale    (D) PD di giunto ×4  ◄── q_ref(t) (profili smoothstep)
   θ_ref=4,73° ──►      θ, θ̇, x, ẋ  →  wheel_u (modo comune)        │
                                │                                   ▼
   traiettoria planare ──► (C) guida laterale → ψ_cmd ──► (B) PD di rotta → yaw_u (modo differenziale)
                                │                                   │
                                └──────► τ_l = 18·clamp(u − yaw_u),  τ_r = 18·clamp(u + yaw_u) ◄──┘
```

- **(A) Bilanciamento e posizione**, modo comune. Una sola legge statica su quattro stati. Non è una cascata: posizione e velocità dell'asse entrano **in parallelo** all'inclinazione nella stessa somma.
- **(B) Rotta**, modo differenziale. PD sull'errore di rotta. Attivo **solo** con la traiettoria planare; nel bilanciamento semplice e nel salto l'imbardata è ad anello aperto (`yaw_u = 0`, riga 639).
- **(C) Guida laterale**, esterno a (B). Trasforma l'errore laterale in una correzione della rotta di riferimento. Insieme a (B) forma l'unica vera cascata del controllore.
- **(D) Gambe.** Quattro PD di giunto indipendenti (anca e ginocchio, due lati) in spazio giunti, con riferimenti generati dal supervisore.

Le gambe e le ruote non si scambiano informazioni se non attraverso la dinamica del robot: (A) assume gambe rigide nella posa comandata.

**Sensori.** Tutto viene dalla ground truth: posa del torso da `/rotino/odom`, baricentro e posizione delle ruote dalla cinematica diretta sull'URDF. `/joint_states` e `/rotino/odom` vengono accoppiati per timestamp (`_try_control`, riga 399) perché a 500 Hz uno sfasamento di un passo fra posa e giunti rende rumorosa la derivata dell'inclinazione. Le derivate ($\dot\theta$, $\dot x$) sono differenze finite **non filtrate**.

### 3.2 Anello (A): bilanciamento

**Definizione dell'angolo.** Il PID definisce
$$\theta_{PID} = -\operatorname{atan2}\big(\Delta x - x_0,\ \Delta z\big)$$
con $\Delta x, \Delta z$ la posizione del baricentro totale rispetto al punto medio delle ruote lungo la direzione di marcia, e $x_0$ = `com_x_ref` = 0,01096 m, il valore di $\Delta x$ nella posa a giunti nulli e torso orizzontale (righe 240-243). $\theta_{PID} = 0$ corrisponde quindi alla posa a torso orizzontale, **non** all'equilibrio.

**Offset di equilibrio.** In equilibrio il baricentro sta sulla verticale dell'asse, $\Delta x = 0$, e $\theta_{PID} = -\operatorname{atan2}(-x_0, \Delta z_0) = \operatorname{atan}(0{,}01096/0{,}13343) = 4{,}694°$. `PITCH_OFFSET` = 4,73° (riga 69) coincide con l'equilibrio statico entro 0,036°. **[C]** Per il §1.3 lo stesso angolo vale per il solo corpo superiore (4,69° in tabella). Il riferimento $\theta_{ref} = 4{,}73°$ significa quindi "baricentro sopra l'asse". L'errore residuo di 0,036° è compensato a regime da un piccolo spostamento di posizione, perché la legge ha un termine in $x$.

**Legge.** Con i guadagni in unità normalizzate e $u\in[-1,1]$ scalato per `WHEEL_GEAR` = 18 N·m:
$$u_{raw} = k_\theta(\theta_{PID}-\theta_{ref}) + k_{\dot\theta}\dot\theta_{PID} - k_{\dot x}(\dot x - \dot x_{ref}) - k_x(x-x_{ref}),\qquad \tau_c = 18\cdot(-u_{raw}) .$$
Posto $\theta_{PID} - \theta_{ref} \simeq -\theta$, con $\theta$ l'inclinazione fisica del §0, la coppia per ruota diventa
$$\tau_c = 18\,\big[k_\theta\theta + k_{\dot\theta}\dot\theta + k_{\dot x}(\dot x-\dot x_{ref}) + k_x(x-x_{ref})\big].$$
Ha la stessa struttura e gli stessi segni della riga sagittale dell'LQR ($U = -KX$ con $K$ negativo, §4.3). **[D]**

**Guadagni in unità fisiche e confronto con l'LQR dell'MPC** (per ruota, posa nominale) **[C]**:

| | $x$ [N·m/m] | $\theta$ [N·m/rad] | $\dot x$ [N·m·s/m] | $\dot\theta$ [N·m·s/rad] | saturazione |
|---|---|---|---|---|---|
| PID (`K_X`, `K_THETA`, `K_X_D`, `K_THETA_D`) × 18 | 14,4 | 23,4 | 2,88 | 3,06 | 6,3 N·m (`MAX_WHEEL` 0,35) |
| TV-LQR dell'MPC, $l = 0{,}163$ m | 3,87 | 20,28 | 5,64 | 2,78 | 10 N·m |

I guadagni su $\theta$ e $\dot\theta$ sono vicini: la stabilizzazione del pendolo è simile. Sulla traslazione il PID è molto diverso: rigidezza di posizione 3,7 volte più alta e smorzamento di velocità la metà.

**Poli ad anello chiuso** sul VL-WIP linearizzato in posa nominale, sagittale, a tempo continuo **[C]**:

| | poli |
|---|---|
| PID | $-253$; $-7{,}62$; $\mathbf{-0{,}149\pm 2{,}137j}$ |
| TV-LQR | $-179$; $-8{,}13$; $-1{,}08\pm0{,}68j$ |

Il PID ha una coppia di poli lenti con smorzamento $\zeta = 0{,}069$ e periodo di 2,9 s: il modo di traslazione è quasi non smorzato. È la conseguenza diretta del rapporto $k_x/k_{\dot x}$. Il modello lineare, da solo, prevede un'oscillazione di avanti-indietro lenta e persistente, in cui l'inclinazione segue l'accelerazione. È coerente con le oscillazioni di ±3-7° dell'originale MuJoCo **[E]**, ma quella misura include anche le gambe cedevoli, che il modello a gambe rigide non contiene.

**Guadagni per fase** (`balance_gains_for_state`, riga 173), in N·m per ruota **[C]**:

| Fase | $k_\theta$ | $k_{\dot\theta}$ | $k_{\dot x}$ | $k_x$ | sat. |
|---|---|---|---|---|---|
| BALANCE | 23,4 | 3,06 | 2,88 | 14,4 | 6,3 |
| PRELOAD | 23,4 | 3,96 | 3,60 | 7,2 | 6,3 |
| THRUST | 23,4 | 4,50 | 3,24 | 1,44 | 6,3 |
| FLIGHT | 23,4 | 1,80 | 0 | 0 | 1,8 |
| LANDING | 23,4 | 3,24 | 1,80 | 0 | 3,96 |
| RECOVERY | 23,4 | 3,60 | 2,16 | 2,88 | 5,04 |
| SETTLE | 23,4 | 3,06 | 3,96 | 2,16 | 6,3 |

Logica della tabella:
- $k_\theta$ non cambia mai: la stabilizzazione dell'instabile non si negozia.
- Durante lo squat e la spinta (PRELOAD, THRUST) si aumenta lo smorzamento di $\dot\theta$ e si riduce il richiamo in posizione. Il moto verticale delle gambe perturba $\theta$, e un richiamo di posizione rigido convertirebbe quel disturbo in accelerazioni orizzontali.
- In volo la coppia è limitata a 1,8 N·m e non c'è richiamo di traslazione: le ruote non toccano terra e ogni coppia si scarica sul beccheggio del corpo in aria.
- In atterraggio si azzera $k_x$ e si ammette lo scivolamento. Il riferimento di posizione viene poi riallineato al punto di atterraggio (`x_ref_active = x`, riga 716), così il robot non cerca di tornare dove ha spiccato il salto.
- In SETTLE si alza lo smorzamento di velocità e si abbassa la rigidezza di posizione, cioè si lavora sul modo lento poco smorzato individuato sopra.

### 3.3 Anelli (B) e (C): rotta e guida planare

Solo con `planar_enable` (`_planar_tracking`, riga 500). La traiettoria è una curva a S cubica $y(x) = Y(3u^2 - 2u^3)$, $u = x/X$, percorsa con una legge oraria quintica in ascissa curvilinea, per cui velocità e accelerazione sono nulle agli estremi (`planar_trajectory.py`).

**(C) Guida laterale.** Con $e_\perp$ l'errore laterale dell'asse rispetto al punto di riferimento:
$$\psi_{cmd} = \psi_{ref} - \operatorname{atan}(K_{lat}\,e_\perp)\cdot\min\!\big(1,\ |v_{ref}|/0{,}10\big),\qquad K_{lat} = 2\ \text{rad/m}.$$
L'arcotangente limita la correzione a ±90° e per errori piccoli dà 2 rad di rotta per metro di errore. Il fattore di velocità azzera la correzione da fermo: un veicolo differenziale annulla l'errore laterale solo muovendosi (vincolo anolonomo), e da fermo girare sul posto per inseguire un punto laterale non riduce l'errore.

**(B) Rotta.** PD in unità normalizzate, saturato a `MAX_YAW` = 0,15 (2,7 N·m):
$$u_{yaw} = K_\psi(\psi_{cmd}-\psi) + K_\omega(\dot\psi_{ref}-\dot\psi),\qquad K_\psi\cdot18 = 1{,}8\ \text{N·m/rad},\quad K_\omega\cdot 18 = 0{,}45\ \text{N·m·s/rad}.$$
Con $\ddot\varphi = 2b_3\tau_d$ i poli sono le radici di $\sigma^2 + 2b_3\cdot0{,}45\,\sigma + 2b_3\cdot1{,}8$: **$-26{,}7$ e $-4{,}7$ rad/s**, cioè un anello sovrasmorzato **[C]**. Il commento alla riga 82 dichiara che l'attrito di strisciamento delle ruote in curva richiede circa 5 volte il guadagno calcolato sulla sola inerzia **[E]**. Il modello (§1.4) non contiene questo attrito; MPC e SMC lo compensano con un feedforward esplicito (§4.4).

L'errore lungo la traiettoria e l'errore di velocità sostituiscono $x - x_{ref}$ e $\dot x - \dot x_{ref}$ in (A).

### 3.4 Anello (D): gambe in spazio giunti

Per ciascun giunto (`_publish_legs`, riga 454):
$$\tau = \operatorname{sat}_{60}\Big(\operatorname{sat}_{60}\big(k_p(q_{ref}-q)\big) + \operatorname{sat}_{8}\big(-k_v\dot q\big)\Big)$$
con $k_p$ = 160 (anca), 200 (ginocchio) N·m/rad e $k_v$ = 12, 14 N·m·s/rad.

**Rigidezza equivalente al piede.** Nella posa nominale $K_x = J^{-T}\operatorname{diag}(160, 200)J^{-1}$ vale, in N/m per gamba **[C]**:
$$K_x = \begin{bmatrix} 4736 & -4736\\ -4736 & 28417\end{bmatrix}.$$
In verticale è circa 24 volte più rigida della molla virtuale dello SMC (1200 N/m, §5.5). Le gambe del PID sono pensate come quasi rigide: il PID muove il robot per **posizioni di giunto**, non per forze.

**Perché lo smorzamento è saturato a ±8 N·m.** Nel controllore MuJoCo originale questo PD girava dentro il simulatore a ogni passo di fisica (2 kHz). Portato nel nodo gira a 500 Hz. La diagnosi (`diagnostics/06_diagnosi.txt`) ha stabilito la causa del "calcio" al rilascio **[E]**. Con la coppia massima di 60 N·m e un'inerzia riflessa al ginocchio di circa $7\cdot10^{-3}$ kg·m² (gamba scarica: stinco più ruota attorno al ginocchio), l'accelerazione massima è $\alpha = 8570$ rad/s². In un periodo di campionamento la velocità può crescere di $\alpha\Delta t$: 17,1 rad/s a 500 Hz contro 4,3 rad/s a 2 kHz. In pochi campioni si arriva a 70 rad/s, dove $k_v\dot q = 840$ N·m, 14 volte il limite. Il termine di smorzamento satura e cambia segno a ogni campione: è un **bang-bang a tempo discreto**, non un'instabilità dei guadagni in tempo continuo.

Il rimedio limita l'energia che un picco di velocità può iniettare in un periodo senza togliere autorità al termine proporzionale. La scansione riportata nella diagnosi **[E]** mostra che abbassare $k_v$ peggiora sempre (fino alla caduta) e che la saturazione del solo smorzamento a 8 N·m dà il risultato migliore (11,5° medi). Il robot resta comunque a un'inclinazione media di circa 11° contro i circa 5° dell'originale. La diagnosi attribuisce il residuo all'anello (A), tarato assumendo gambe servoassistite a 2 kHz: con gambe più cedevoli il sistema visto dalle ruote cambia. Questa ipotesi non è verificata.

### 3.5 Supervisore del salto

Sette stati: BALANCE → PRELOAD → THRUST → FLIGHT → LANDING → RECOVERY → SETTLE → BALANCE.

| Stato | Riferimenti di giunto (anca, ginocchio) | Uscita |
|---|---|---|
| PRELOAD | da (0, 0) a (+0,12, −0,22) con smoothstep in 0,80 s: accucciata | dopo 0,80 s |
| THRUST | da PRELOAD a (−0,27, +0,44) con $1-(1-\tau)^3$ in 0,02 s: estensione brusca | decollo, o 0,16 s senza decollo → LANDING |
| FLIGHT | posa congelata al decollo | contatto confermato |
| LANDING | dalla posa di contatto a (+0,08, −0,20) in 0,80 s: gambe cedevoli | dopo 0,80 s |
| RECOVERY | ritorno a (0, 0) in 2,0 s | dopo 2,0 s |
| SETTLE | (0, 0) | assestamento per 0,30 s o 12 s di attesa |

- **Profili.** Lo smoothstep $3\tau^2-2\tau^3$ ha velocità nulla agli estremi e non chiede coppie a gradino. Il profilo di spinta $1-(1-\tau)^3$ ha velocità **massima all'inizio**: l'estensione più rapida possibile nel tempo di spinta.
- **Decollo.** È confermato solo se valgono insieme quattro condizioni: contatto perso (isteresi), forza normale sotto soglia, distacco geometrico della ruota (sia assoluto > 0,5 mm sia relativo alla quota di inizio salto > 3 mm) e velocità verticale del baricentro > 0,08 m/s. La congiunzione esclude i falsi decolli dovuti a rimbalzi o a scarichi momentanei.
- **Assestamento.** Tutte le condizioni devono valere per 0,30 s: |errore di inclinazione| < 2°, |velocità di beccheggio| < 8°/s, |velocità| < 0,04 m/s, |comando ruote| < 0,06 e |errore di posizione| < 1,5 cm.

---

## 4. MPC + TV-LQR + VMC

`src/rotino_mpc/rotino_mpc/controller.py` e `solvers.py`. Implementa l'architettura della Fig. 4 del paper.

```
 riferimenti (s, ṡ, z, ż sull'orizzonte) ──► MPC corpo superiore, 100 Hz ──► Δs, F_z
                                              │ piano del baricentro x_h(t+Δt)
                                              ▼
 stima (s, θ, φ, ṡ, θ̇, φ̇) ──► TV-LQR K(l), 500 Hz ──► τ_l, τ_r (ruote)
                                  X_ref = [s_plan − Δs, atan2(Δs, z_ref), φ_ref, ṡ_plan, 0, φ̇_ref]
 Δs, z_ref, F_z ──────────────► VMC task-space, 500 Hz ──► τ_hip, τ_knee (gambe)
```

### 4.1 Perché questa architettura: le motivazioni del paper

Il paper parte da una constatazione (Sez. 1). I controllori per robot bipedi su ruote pubblicati fino ad allora si dividevano in due gruppi:

- **pendolo inverso su ruote puro** (LQR con feedforward, UDE, IDA-PBC): semplici, ma ignorano l'influenza del torso sul moto;
- **whole-body control** (Ascento, Xin et al.): completi, ma complessi da modellare e da calcolare.

La proposta è intermedia: **disaccoppiare** il robot in due modelli semplici e dare a ciascuno il controllore più adatto.

1. **VL-WIP + TV-LQR per l'equilibrio.** L'equilibrio è un problema veloce, instabile e ben descritto da un modello lineare a parametri variabili (la lunghezza $l$ cambia con l'altezza). Un LQR ricalcolato in funzione di $l$ dà la miglior controreazione lineare per un compromesso dichiarato fra errore e sforzo di controllo (eq. 15). Può girare alla frequenza più alta perché è una moltiplicazione matrice-vettore.
2. **Massa concentrata + MPC per il corpo superiore.** Dove mettere il baricentro rispetto alle ruote e quanta forza verticale chiedere alle gambe è un problema più lento, ma **vincolato**:
   - il baricentro non può scostarsi dal contatto oltre il cono d'attrito (eq. 18: $|\Delta s|\le\mu h$, approssimazione a piramide del cono);
   - le gambe hanno uno spazio di lavoro finito ($|\Delta s|\le\sqrt{L_{max}^2-z_b^2}$);
   - la forza verticale è limitata ($F_{min}\le F_z\le F_{max}$): $F_{min} > 0$ mantiene il contatto, $F_{max}$ rispetta gli attuatori.

   L'MPC tratta i vincoli direttamente nell'ottimizzazione invece di saturare a posteriori. Inoltre **predice** l'evoluzione dello stato su un orizzonte, quindi reagisce ai riferimenti futuri prima che arrivino.
3. **VMC per tradurre in coppie di giunto.** Il VMC (eq. 19) realizza $\Delta s$ e $F_z$ come molla-smorzatore virtuale al piede più una forza in avanti. Non serve la dinamica inversa completa delle gambe in appoggio.
4. **Filtro di Kalman lineare** invece dell'EKF di Bloesch et al., ritenuto troppo costoso (Sez. 4.3).

Il paper sostiene l'uso dell'MPC con la letteratura citata nell'introduzione:
- permette di integrare facilmente i vincoli e di migliorare la stabilità, prevedendo il tempo di volo e la sottoattuazione (rif. 21-22);
- un controllo con anteprima compensa l'errore di ZMP dovuto alla differenza fra modello semplice e modello multicorpo (Kajita et al., rif. 23);
- il controllo ZMP con anteprima migliora la reiezione dei disturbi nei bipedi (Wieber, rif. 24), "cosa cruciale anche per i WBR".

La motivazione esplicita della Sez. 4.2 è: *"in order to improve the robustness of control, MPC is used to solve the horizontal displacement of the CoM and the vertical force. It can predict the state variation of the WBR in a longer time frame."*

### 4.2 Vantaggi concreti nell'implementazione di RoTino

Quanto segue deriva dal codice, non dal paper.

**(a) Anticipo sui riferimenti in un sistema a fase non minima.** L'MPC riceve i riferimenti $s(t_i), \dot s(t_i), z(t_i), \dot z(t_i)$ per tutti i 25 nodi dell'orizzonte (`_run_mpc`, riga 703). Il robot deve inclinarsi *prima* di accelerare (§1.6) e l'inclinazione ha un limite (§4.5). Vedere il riferimento con 0,5 s di anticipo permette di distribuire lo spostamento del baricentro e di iniziarlo prima della variazione del riferimento. L'LQR, da solo, reagisce solo all'errore presente. Con un profilo di velocità noto la differenza è strutturale.

**(b) Vincoli rispettati nel piano, non solo nel comando.** Il limite su $\Delta s$ entra nella QP. Il piano che l'MPC restituisce, cioè la traiettoria predetta del baricentro, rispetta quindi il limite su tutto l'orizzonte. L'LQR insegue quel piano (`s_plan`, righe 642-647) e non il riferimento grezzo: il riferimento dell'anello veloce è già compatibile con i limiti fisici. Lo SMC invece satura $\theta^*$ a posteriori (§5.2) e deve gestire il windup con logica ad hoc.

**(c) Un'unica sintesi per la forza verticale.** $F_z$ viene pianificata con lo stesso modello e con limiti che dipendono dalla fase: in spinta il tetto sale da 3 a 6 volte il peso. Lo stesso blocco gestisce appoggio, variazione di quota e spinta del salto.

**(d) Separazione dei tempi.** 100 Hz bastano: il tempo caratteristico del modello orizzontale è $\sqrt{h/g} = 0{,}13$ s, cioè 13 intervalli dell'MPC. La parte instabile e veloce resta all'LQR a 500 Hz.

**Costi e limiti:**
- **Nessuna azione integrale.** Né l'LQR né l'MPC hanno stati integrali o un modello del disturbo. Sul modello ridotto, un disturbo persistente lascia al solo TV-LQR un errore di posizione a regime: 0,65 m con 3 N costanti sul torso, 0,08 m con 0,3 N·m di bias per ruota **[S-R]**, §6.6. Per l'MPC completo non l'ho misurato, ma nulla nella sua formulazione annulla quell'errore.
- **Modello di predizione povero.** Il corpo superiore è un punto, senza dinamica di beccheggio. L'MPC non sa che per spostare il baricentro di $\Delta s$ il pendolo deve prima essere portato all'angolo corrispondente: lo delega all'LQR.
- **Costo di calcolo.** Due QP a 25 variabili ogni 10 ms, risolte con 60 iterazioni di gradiente accelerato.

### 4.3 Differenze fra paper e implementazione

| Aspetto | Paper | `rotino_mpc` |
|---|---|---|
| Frequenze (Fig. 4) | MPC 100 Hz, stimatore 500 Hz, TV-LQR e task-space 1000 Hz | MPC 100 Hz, tutto il resto 500 Hz |
| $I_y$ | $\tfrac13 m_b l^2$ (Tabella 1) | reale dall'URDF, schedulato con la posa |
| Discretizzazione MPC | Eulero: $A_k = I + A_c\Delta T$, $B_k = B_c\Delta T$ | esatta (ZOH) per il doppio integratore: $B_k = [\Delta T^2/2,\ \Delta T]^T$ |
| Struttura della QP | un problema sullo stato a 5 componenti dell'eq. 9 | due QP disaccoppiate (orizzontale, verticale) |
| Vincolo su $\Delta s$ | $\min(\mu h,\ \sqrt{L_{max}^2-z_b^2})$ | come il paper **più** un tetto fisso di 0,03 m |
| Solutore | QP condensata (rif. 28) | QP condensata, solo vincoli di box, FISTA |
| Feedforward VMC | dinamica inversa in volo | $-F_z/2$ lungo la verticale e compensazione dello smorzamento URDF; nessuna dinamica inversa |
| Kalman | un filtro 2D | tre filtri scalari (equivalente) |
| Imbardata | solo tramite la riga di $\varphi$ del TV-LQR | TV-LQR con peso differenziale separato, feedforward d'attrito e integrale |

Nell'eq. 9 l'unico accoppiamento fra orizzontale e verticale è il coefficiente $(g+\ddot z)/h$. Congelandolo sull'orizzonte con il valore del riferimento, i due blocchi diventano indipendenti e ciascuno è lineare tempo-invariante. Il paper lo tratta implicitamente allo stesso modo, perché anche lì la matrice $B_c$ contiene $\ddot z$ e $h$.

### 4.4 TV-LQR (eq. 15-16)

**Formulazione.** $J = \int_0^\infty(\tilde X^TQ\tilde X + U^TRU)\,dt$, $U = -K(l)\tilde X$. $K$ si ottiene dall'equazione algebrica di Riccati, risolta come sottospazio stabile dell'Hamiltoniana (`care`, `solvers.py:15`).

**Pesi** (`controller.py:41-48`):
$$Q = \operatorname{diag}(30,\ 400,\ 80,\ 15,\ 6,\ 2)\quad\text{su}\ (s,\ \theta,\ \varphi,\ \dot s,\ \dot\theta,\ \dot\varphi),\qquad R = T^T\operatorname{diag}(2,\ 100)\,T,\ \ T = \begin{bmatrix}\tfrac12&\tfrac12\\-\tfrac12&\tfrac12\end{bmatrix}.$$

- **Lettura di $Q$ alla Bryson.** $1/\sqrt{q_{ii}}$ è lo scostamento "equivalente" a un'unità di costo: 0,18 m in $s$, 0,050 rad (2,9°) in $\theta$, 0,11 rad in $\varphi$, 0,26 m/s in $\dot s$, 0,41 rad/s in $\dot\theta$, 0,71 rad/s in $\dot\varphi$. L'inclinazione ha la priorità, la posizione è accettata a decine di centimetri: il robot "cede" in traslazione per non cadere.
- **Lettura di $R$.** $T$ trasforma $(\tau_l, \tau_r)$ in $(\tau_c, \tau_d)$, per cui $U^TRU = 2\tau_c^2 + 100\tau_d^2$. Con $R = I$ il costo sarebbe $\tau_l^2+\tau_r^2 = 2\tau_c^2 + 2\tau_d^2$. Il peso sul modo comune coincide quindi con $R = I$, come dichiara il commento a riga 45, mentre il modo differenziale è penalizzato 50 volte di più. Il modello è disaccoppiato (§1.6) e con questa $R$ anche il costo lo è: la Riccati si separa in un problema sagittale e uno di imbardata, e i due pesi agiscono ciascuno solo sulla propria parte.

**Perché il differenziale è penalizzato così.** Il commento (righe 42-44) motiva la scelta con una risonanza torsionale delle gambe in appoggio, a circa 76 rad/s (12 Hz): l'inerzia di imbardata del torso sulle molle virtuali del VMC **[E]**. Un anello di imbardata veloce la eccita in un ciclo limite permanente. Con $R_d = 100$ l'anello di imbardata ha pulsazione di attraversamento **15,4 rad/s [C]**, un quinto della risonanza. Il modello VL-WIP non contiene questa risonanza: il vincolo è stato scoperto in simulazione e imposto attraverso il peso.

**Guadagni e poli** in posa nominale **[C]**:
$$K = \begin{bmatrix} -3{,}873 & -20{,}28 & -0{,}894 & -5{,}637 & -2{,}784 & -0{,}214\\ -3{,}873 & -20{,}28 & +0{,}894 & -5{,}637 & -2{,}784 & +0{,}214\end{bmatrix}$$
Poli: $-179{,}5$; $-8{,}13$; $-1{,}08\pm0{,}68j$ (sagittali); $-7{,}45\pm2{,}62j$ (imbardata). Il polo a $-179$ rad/s sta oltre la banda del filtro su $\dot\theta$ ($1/8{,}96\ \text{ms} = 112$ rad/s). Nel sistema reale quel modo è determinato dal filtro e dal campionamento più che dall'LQR. $-8{,}1$ è la dinamica di inclinazione. La coppia lenta $-1{,}08\pm0{,}68j$ ($\zeta = 0{,}85$) è il ritorno in posizione: è la dinamica che si osserva dopo una spinta.

**Scheduling su $l$.** `LQRSchedule` (`solvers.py:33`) calcola $K$ sui 25 punti della griglia di pose, ciascuno con il proprio $I_y$, e interpola linearmente in $l$ a ogni passo. Lungo la griglia i poli sagittali restano quasi fermi **[C]**:

| $l$ [m] | $K_\theta$ [N·m/rad] | $K_{\dot\theta}$ | $K_{\dot s}$ | poli sagittali |
|---|---|---|---|---|
| 0,219 | 21,22 | 3,13 | 5,51 | $-139{,}9$; $-8{,}06$; $-1{,}15\pm0{,}68j$ |
| 0,163 | 20,28 | 2,78 | 5,64 | $-179{,}5$; $-8{,}13$; $-1{,}08\pm0{,}68j$ |
| 0,074 | 18,43 | 2,33 | 6,32 | $-281{,}1$; $-8{,}16$; $-0{,}86\pm0{,}63j$ |

È lo scopo del TV-LQR: comportamento ad anello chiuso invariante rispetto all'altezza. $K_s = -3{,}873$ non cambia con $l$ e coincide numericamente con $\sqrt{q_s/r_c} = \sqrt{30/2}$. Allo stesso modo il guadagno di imbardata vale $0{,}894 = \sqrt{q_\varphi/r_d} = \sqrt{80/100}$. I poli d'imbardata non dipendono da $l$ perché $I_z$ è tenuto costante. L'interpolazione a $l$ congelato è giustificata finché $l$ varia lentamente rispetto alla dinamica di anello chiuso. Durante la spinta del salto questo non è garantito, e il modello ignora i termini in $\dot l$.

**Saturazione e riferimento.** Coppia per ruota ±10 N·m (contro i 18 dell'URDF), con priorità al modo comune: il differenziale è limitato a ±2,5 N·m e il comune a $\pm(10-|\tau_d|)$ (righe 658-660). Il riferimento è
$$X_{ref} = \big[s_{plan}-\Delta s,\ \operatorname{atan2}(\Delta s, z_{ref}),\ \varphi_{ref},\ \dot s_{plan},\ 0,\ \dot\varphi_{ref}\big].$$
$s_{plan}, \dot s_{plan}$ sono lo stato del baricentro predetto dall'MPC al nodo successivo, interpolato fra due soluzioni (il "virtual CoM" della Fig. 4). L'asse deve stare $\Delta s$ dietro il baricentro pianificato e il pendolo all'angolo che realizza quello spostamento.

### 4.5 Imbardata: feedforward d'attrito e integrale

Oltre alla riga di $\varphi$ del TV-LQR (righe 650-660):
$$\tau_d \mathrel{+}= \underbrace{0{,}25\tanh\!\big(\dot\varphi_{ref}/0{,}01\big) + 0{,}10\,\dot\varphi_{ref}}_{\text{attrito di strisciamento}} + \underbrace{\operatorname{sat}_{0,6}\Big(-0{,}6\!\int\! e_\varphi\,dt\Big)}_{\text{integrale}} .$$

- Il **feedforward** compensa la coppia di strisciamento delle ruote in curva, modellata come Coulomb (0,25 N·m) più viscoso (0,10 N·m per rad/s), con valori identificati in Gazebo **[E]**. La $\tanh$ con 0,01 rad/s rende continua la funzione segno. Dipende dal **riferimento** e non dalla misura, quindi non può generare chattering.
- L'**integrale** annulla l'errore statico di rotta dovuto all'attrito residuo. Si attiva solo con errore sopra 0,035 rad o durante una sterzata comandata, per evitare la caccia da stick-slip su errori piccoli, ed è saturato a 0,6 N·m.
- L'errore di rotta è **saturato a 0,35 rad** e il riferimento teleoperato non può anticipare il robot di più di 0,35 rad (anti-windup del riferimento, righe 334-337).

### 4.6 MPC del corpo superiore (eq. 17-18)

**Modelli di predizione** (`UpperBodyMPC`, `solvers.py:79`), $\Delta T = 0{,}02$ s, $N = 25$ (orizzonte di 0,5 s):
$$\text{orizzontale:}\ \begin{bmatrix}s\\\dot s\end{bmatrix}_{k+1} = \begin{bmatrix}1&\Delta T\\0&1\end{bmatrix}\begin{bmatrix}s\\\dot s\end{bmatrix}_k + \begin{bmatrix}\Delta T^2/2\\\Delta T\end{bmatrix}\frac{g+\ddot z_{ref}}{h}\,\Delta s_k$$
$$\text{verticale:}\ \begin{bmatrix}z\\\dot z\end{bmatrix}_{k+1} = \begin{bmatrix}1&\Delta T\\0&1\end{bmatrix}\begin{bmatrix}z\\\dot z\end{bmatrix}_k + \begin{bmatrix}\Delta T^2/2\\\Delta T\end{bmatrix}\frac{F_{z,k}}{m_b} - \begin{bmatrix}\Delta T^2/2\\\Delta T\end{bmatrix}g$$
Lo stato orizzontale è quello del **baricentro** ($s_{com} = s + S_C$, $\dot s_{com}$ da $V_{com}$), il verticale è $z = Z_C$ con $\dot z$ da $V_{com}$.

**Condensazione.** Con $x_i = \Phi_i x_0 + \Gamma_i U + C_i$ (`_condense`, `solvers.py:61`) il problema diventa
$$\min_U\ \tfrac12 U^THU + g^TU,\quad H = \Gamma^T\bar S\Gamma + W I,\quad g = \Gamma^T\bar S(\Phi x_0 + C - x_{ref}),\qquad lo\le U\le hi .$$
Solo vincoli di box, per cui la proiezione è una saturazione componente per componente e basta un gradiente proiettato accelerato (FISTA, `box_qp`, `solvers.py:48`): 60 iterazioni, passo $1/\lambda_{max}(H)$, convergenza $O(1/k^2)$. La soluzione precedente, traslata di un nodo, fa da punto di partenza.

**Pesi.**
- *Orizzontale*: $S_h = \operatorname{diag}(50, 20)$ su $(s, \dot s)$, $W_h = 200$ su $\Delta s$. Uno spostamento di 1 cm costa $200\cdot10^{-4} = 0{,}02$ per nodo, quanto un errore di posizione di 2 cm ($50\cdot 4\cdot10^{-4}$). L'MPC accetta di sbilanciare il baricentro solo per errori di posizione più grandi di circa il doppio. Con un gradino di 0,1 m sul riferimento il primo $\Delta s$ vale 0,025 m, sotto il tetto **[C]**.
- *Verticale*: $S_v = \operatorname{diag}(5000, 150)$ su $(z, \dot z)$, $W_v = 2\cdot10^{-3}$ su $F_z$. Il peso agisce su $F_z$ **assoluto** e non sullo scostamento dal peso, quindi in linea di principio spinge $F_z$ sotto $m_bg$. In anello chiuso sul modello perfetto l'effetto è trascurabile: errore di quota a regime 0,01 mm, $F_z = 34{,}629$ N contro un peso di 34,629 N **[C]**. $S_v$ è 100 volte $S_h$: la quota va tenuta rigidamente perché in appoggio il VMC non ha una molla verticale (§4.7).

**Vincoli.**
$$|\Delta s| \le \min\big(\mu z,\ \sqrt{L_{max}^2-z_b^2},\ 0{,}03\big),\qquad 0{,}3\,m_bg\le F_z\le \{3;\ 6\}\,m_bg .$$
In posa nominale $\mu z = 0{,}357$ m e $\sqrt{L_{max}^2 - z_b^2} = 0{,}184$ m. **Il vincolo attivo è sempre il tetto di 0,03 m** (`DS_MAX_CAP`), che non viene dal paper. Il limite d'attrito del paper ($\mu h$ con $\mu = 2{,}2$) è irrealistico come limite di inclinazione, perché permetterebbe $\operatorname{atan}(2{,}2) = 65°$. Il tetto di 3 cm corrisponde a $\theta_{ref,max} = \operatorname{atan}(0{,}03/0{,}162) = 10{,}48°$ **[C]**, entro la zona in cui il VL-WIP linearizzato resta credibile. Per il §1.6 questo tetto fissa anche la massima decelerazione a regime: $g_{st}\cdot0{,}03/0{,}162 = 1{,}13$ m/s². $F_{min} = 0{,}3\,m_bg$ impedisce all'MPC di pianificare uno scarico che farebbe perdere il contatto.

**Da $\Delta s$ all'LQR e alle gambe.** $\Delta s$ viene filtrato ($\alpha = 0{,}9$, 19 ms) e usato due volte:
- nell'LQR, come angolo di riferimento $\theta_{ref} = \operatorname{atan2}(\Delta s, z_{ref})$ e come arretramento dell'asse rispetto al baricentro pianificato;
- nel VMC, come posizione orizzontale desiderata della ruota rispetto al baricentro, $p_{d,x} = c_{b,x} - \Delta s$.

Le ruote (inclinando il pendolo) e le gambe (spostando il piede sotto il corpo) realizzano lo stesso spostamento in modo coerente.

### 4.7 VMC (eq. 19)

Per ciascuna gamba (`_vmc`, riga 452):
$$\tau = J^T\big[K_p(p_d - p_f) + K_d(v_d - v_f) + F_{ff}\big] + b_{leg}\,\dot q,\qquad F_{ff} = -\tfrac12F_z\,\hat z_b$$
con $p_d = (c_{b,x} - \Delta s,\ c_{b,z} - z_{ref})$ e $v_d = (0, -\dot z_{ref})$ nella terna `base_link`. $\hat z_b$ è la verticale mondo espressa in terna corpo. $F_{ff}$ è la forza che ciascun piede esercita sul suolo: metà del sostegno pianificato, diretta verso il basso. Il termine $b_{leg}\dot q$ annulla lo smorzamento viscoso dei giunti dell'URDF (0,8 N·m·s/rad), che altrimenti si sommerebbe al $K_d$ virtuale in modo dipendente dalla configurazione.

**Guadagni per fase** (righe 78-84):

| Fase | $K_p$ [N/m] (x, z) | $K_d$ [N·s/m] (x, z) | Sostegno verticale |
|---|---|---|---|
| HOLD (ancorato) | 1500, 2000 | 40, 50 | molla |
| Appoggio con MPC | **1500, 0** | 40, 15 | **solo $F_z$ dall'MPC** |
| Appoggio senza MPC | 1500, 2000 | 40, 50 | molla |
| Volo | 800, 800 | 20, 20 | nessuno (gambe raccolte) |

La scelta chiave è $K_{p,z} = 0$ in appoggio. La quota è regolata dall'MPC attraverso $F_z$ a 100 Hz. Una molla verticale in parallelo sarebbe un secondo regolatore sulla stessa grandezza e si opporrebbe alle variazioni di quota pianificate. Resta uno smorzamento di 15 N·s/m contro le oscillazioni fra due aggiornamenti dell'MPC. In orizzontale la rigidezza di 1500 N/m con 40 N·s/m ha costante di superficie $K_p/K_d = 37{,}5$ s⁻¹, cioè 27 ms: il piede converge alla posizione desiderata molto più in fretta del moto del baricentro. Come ordine di grandezza, con metà di $m_b$ per gamba, $\omega_n = \sqrt{1500/1{,}765} = 29$ rad/s e $\zeta = 0{,}39$ **[C]**.

### 4.8 Salto

Macchina a stati: BALANCE → PRELOAD → THRUST → FLIGHT → LANDING → BALANCE. Il salto parte solo a robot fermo ($|\dot\theta| < 0{,}3$ rad/s, $|\dot s| < 0{,}08$ m/s). Le quote vengono dalla tabella del §1.2 **[C]**: accucciata con distanza anca-asse 0,12 m ($Z_C = 0{,}104$ m), obiettivo di spinta 0,24 m ($Z_C = 0{,}213$ m), volo 0,17 m ($Z_C = 0{,}150$ m).

- **PRELOAD**: smoothstep da $z_{stand}$ a $z_{low}$ in 0,8 s.
- **THRUST**: accelerazione costante $a = v_{to}^2/\big(2(z_{high}-z_{low})\big)$, così il baricentro arriva all'estensione con la velocità di decollo richiesta. Con $v_{to} = 1{,}2$ m/s: $a = 6{,}62$ m/s² per 0,181 s. $F_z = m_b(g+a) = 58{,}0$ N $= 1{,}67\,m_bg$, dentro il tetto di spinta di $6\,m_bg$ **[C]**. Il termine $\ddot z_{ref}$ entra anche nel coefficiente $(g+\ddot z)/h$ del modello orizzontale: durante la spinta lo stesso $\Delta s$ produce un'accelerazione orizzontale 1,67 volte più grande, e l'MPC ne tiene conto.
- **FLIGHT**: ruote frenate ($-0{,}05\,\dot q$), gambe portate sotto il baricentro con il VMC di volo, MPC e LQR sospesi.
- **LANDING**: all'atterraggio l'MPC viene azzerato, il riferimento di posizione riallineato alla posizione attuale e la quota riportata con smoothstep in 1,0 s, più 1,5 s di assestamento.

---

## 5. Cascata sliding mode

`src/rotino_smc/rotino_smc/controller.py`. Sostituisce MPC, TV-LQR e VMC con una cascata in cui la parte sliding mode agisce sul beccheggio e sulle gambe, mentre traslazione e imbardata restano lineari. Stimatore, macchina a stati, riferimenti e teleoperazione sono identici all'MPC (§2, §4.8).

### 5.1 Struttura

```
 s_ref, ṡ_ref ─► v_ref = ṡ_ref + K_POS(s_ref − s) ─► PI a guadagni negativi ─► θ* (saturato a ±12°)
                                                                                   │
 θ, θ̇, l ───────────────────────────────────► super-twisting su s₁ = θ̇ + c₁(θ − θ*) ─► τ_c
 φ, φ̇, φ_ref ────────────────────────────────► PD + attrito + integrale ─────────────► τ_d
                                                         τ_l = τ_c − τ_d,  τ_r = τ_c + τ_d
 z_ref, ż_ref, z̈_ref ─► F_z = m_b(g + z̈_ref) ─► SMC task-space per gamba ─► τ_hip, τ_knee
```

**Un solo controllore di beccheggio, due controllori di gamba.** Il beccheggio del pendolo equivalente è un unico grado di libertà, azionato dalla somma delle coppie delle ruote. Due controllori sliding mode indipendenti, uno per ruota, avrebbero la **stessa** superficie di scorrimento e due integratori $W$ che si contendono lo stesso errore. Le due gambe invece sono catene cinematiche distinte, ciascuna con il proprio Jacobiano. L'ipotesi di simmetria della Sez. 3 del paper serve a ridurre il robot a un modello sagittale unico, non a duplicare il controllore dell'equilibrio (`docs/Storico_lavoro.md` §2.2).

### 5.2 Anello esterno: posizione → velocità → inclinazione

**Legge** (`_outer_velocity_loop`, riga 503):
$$v_{ref} = \operatorname{sat}_{2}\big(\dot s_{ref} + K_{pos}(s_{ref}-s)\big),\qquad \theta^* = \operatorname{sat}_{\theta_{max}}\Big(K_P(\dot s - v_{ref}) + K_I\!\int(\dot s - v_{ref})\,dt\Big)$$
con $K_{pos} = 1{,}2$ s⁻¹, $K_P = -0{,}30$ rad·s/m, $K_I = -0{,}10$ rad/m, $\theta_{max} = 0{,}21$ rad (12,03°).

**Perché i guadagni sono negativi.** Si assume che l'anello interno sia molto più veloce ($\theta\simeq\theta^*$). Allora, per il §1.6, l'asse accelera con $\ddot s = g_{st}\theta^*$. Con $e_v = \dot s - v_{ref}$:
$$\dot e_v \simeq g_{st}\big(K_Pe_v + K_I{\textstyle\int} e_v\big) - \dot v_{ref}.$$
È stabile solo se $g_{st}K_P < 0$ e $g_{st}K_I < 0$. Dato che $g_{st} > 0$, **servono $K_P, K_I < 0$**. Detto fisicamente: per accelerare in avanti ($e_v < 0$) serve un'inclinazione in avanti ($\theta^* > 0$). Il docstring del codice attribuisce il segno alla fase non minima. Più esattamente, il segno viene dal guadagno statico inclinazione→accelerazione e dalla convenzione $e_v = \dot s - v_{ref}$. La fase non minima (§1.6) è il motivo per cui la velocità si comanda **attraverso** l'inclinazione: una coppia diretta sulle ruote muove l'asse inizialmente nel verso sbagliato rispetto al moto a regime. **[D]**

**Banda.** Con $g_{st} = 6{,}14$: $g_{st}|K_P| = 1{,}84$ rad/s, il valore dichiarato nel commento di riga 49. **[C]** Poli dell'anello quasi statico **[C]**:
- solo PI di velocità: $-1{,}40$ e $-0{,}44$ rad/s;
- con il termine di posizione ($e_v = \dot s + K_{pos}s$ per $s_{ref} = 0$): $-0{,}76\pm1{,}33j$ e $-0{,}31$ rad/s.

Il polo lento a $-0{,}31$ (3,2 s) è quello che domina il ritorno in posizione dopo una spinta (§6.5). È circa 3,5 volte più lento della coppia $-1{,}08\pm0{,}68j$ dell'LQR.

**Saturazione a 12°.** Oltre i 15° circa il VL-WIP linearizzato perde validità (commento riga 51), e l'inversione esatta del super-twisting usa proprio quel modello. Il limite su $\theta^*$ fissa anche la massima accelerazione a regime: $g_{st}\tan 12{,}03° = 1{,}31$ m/s². Anti-windup per integrazione condizionata: si integra solo se $\theta^*$ non è saturo.

**Perché la velocità dell'asse e non del baricentro.** La velocità orizzontale del baricentro differisce da quella dell'asse per la derivata di $S_C$:
$$\dot s_{com} - \dot s_{axle} = \frac{d}{dt}\big(l\sin\theta\big) \simeq l\,\dot\theta\quad(\dot l\approx 0).$$
Durante un'oscillazione di beccheggio questo termine è della stessa grandezza della velocità di marcia: lo storico riporta circa 0,4 m/s in un ciclo di ±6° **[E]**. $\dot\theta$ risponde a $\theta^*$ con i tempi dell'anello **interno**, quindi usare $\dot s_{com}$ crea un percorso veloce dall'uscita dell'anello esterno alla sua stessa misura, che il progetto quasi statico non prevede. Sperimentalmente, con $\dot s_{com}$ il robot entrava oltre circa 0,7 m/s in un ciclo limite permanente che scompariva usando $\dot s_{axle}$ **[E]**. Anche il TV-LQR ha nel suo stato la velocità dell'asse. L'MPC continua invece a usare $V_{com}$ per il proprio modello (§2.3). Nell'MPC la velocità massima teleoperata è 0,6 m/s; nello SMC arriva a 2 m/s, verificato fino a 2 m/s **[E]**.

### 5.3 Beccheggio: super-twisting

**Superficie.**
$$s_1 = \dot\theta + c_1(\theta - \theta^*),\qquad c_1 = 10\ \text{s}^{-1} .$$
Su $s_1 = 0$ l'errore di inclinazione decade come $e^{-c_1t}$, con costante di tempo 0,1 s. È confrontabile con il polo di inclinazione dell'LQR ($-8{,}13$).

**Inversione esatta.** Dal modello linearizzato, $\ddot\theta = a_2\theta + b_2(\tau_l+\tau_r) + d$, con $d$ l'insieme di tutto ciò che il modello non contiene. Imponendo $\tau_l + \tau_r = (\nu - a_2\theta)/b_2$ si ottiene $\ddot\theta = \nu + d$. Nel codice (`_pitch_super_twisting`, riga 520):
$$\tau_c = \frac{\nu - a_2(l)\,\theta}{2\,b_2(l)},\qquad \nu = -c_1\dot\theta - k_a\sqrt{|s_1|}\,\sigma_\varepsilon(s_1) + W,\qquad \dot W = -k_b\,\sigma_\varepsilon(s_1),\qquad \sigma_\varepsilon(s) = \frac{s}{|s|+\varepsilon}.$$
$a_2(l)$ e $b_2(l)$ si ricalcolano a ogni passo con l'$I_y$ della posa, interpolato sulla griglia (§1.5). Il fattore 2 viene da $\tau_c = (\tau_l+\tau_r)/2$. Con $b_2 < 0$ il segno si inverte automaticamente.

**Dinamica della variabile di scorrimento.** Derivando $s_1$ e sostituendo:
$$\dot s_1 = -k_a\sqrt{|s_1|}\,\sigma_\varepsilon(s_1) + W + \underbrace{d - c_1\dot\theta^*}_{\rho(t)} .$$
**[D]** La perturbazione $\rho$ contiene:
- i termini non lineari trascurati ($\sin\theta$ contro $\theta$, termini centripeti);
- i termini in $\dot l$;
- l'errore su $a_2$ e $b_2$;
- l'errore di stima e il ritardo del filtro su $\dot\theta$;
- ogni disturbo esterno che agisce sul beccheggio;
- la derivata del riferimento $\dot\theta^*$, che la legge non compensa.

**Cosa garantisce l'algoritmo ideale.** Con $\sigma_\varepsilon$ sostituita dalla funzione segno, $\dot s_1 = -k_a|s_1|^{1/2}\operatorname{sign}(s_1) + W + \rho$ con $\dot W = -k_b\operatorname{sign}(s_1)$ è il super-twisting di Levant. Se $|\dot\rho|\le L$ e i guadagni soddisfano condizioni sufficienti note, $s_1$ e $\dot s_1$ vanno a zero in tempo finito, qualunque sia $\rho$ entro quella classe. Una condizione **necessaria** è $k_b > L$: se il termine integrale varia più lentamente della perturbazione, $W$ non può inseguirla. Con $k_b = 250$ il controllore può quindi tollerare al più $|\dot\rho| < 250$ rad/s³. Levant (1998) propone per il differenziatore robusto, che ha la stessa struttura, la taratura $\lambda_1 = 1{,}1L$, $\lambda_0 = 1{,}5\sqrt L$. La coppia $(k_a, k_b) = (25, 250)$ corrisponde a $L\approx 227$ da $k_b$ e $L\approx 278$ da $k_a$, due valori coerenti fra loro. **[D]** Non ho trovato nel workspace una stima di $L$ per RoTino: il dimensionamento resta implicito.

**Cosa garantisce la versione implementata.** Due scostamenti dall'algoritmo ideale:
1. **Segno regolarizzato** in **entrambi** i termini, compreso l'integratore. La legge diventa continua. In assenza di ritardi non converge in tempo finito a $s_1 = 0$, ma in un intorno di ampiezza dell'ordine di $\varepsilon = 0{,}02$ rad/s. È la tecnica dello strato limite: si scambia l'esattezza con l'assenza di discontinuità. Con il ritardo del filtro su $\dot\theta$ questo non basta: in Gazebo $s_1$ ha un rms di 3,7 $\varepsilon$ e il controllore oscilla a circa 10 Hz (§6.7).
2. **Guadagno equivalente decrescente.** Il termine proporzionale $k_a\sqrt{|s_1|}\sigma_\varepsilon(s_1)$, diviso per $s_1$, dà un guadagno lineare equivalente che **diminuisce** con l'errore **[C]**:

| $\lvert s_1\rvert$ [rad/s] | 0,02 | 0,1 | 1 | 4 | 7,6 |
|---|---|---|---|---|---|
| $k_a\sqrt{\lvert s_1\rvert}/(\lvert s_1\rvert+\varepsilon)$ [s⁻¹] | 88 | 66 | 24,5 | 12,4 | 9,0 |

Per errori grandi il super-twisting è **meno** aggressivo di un anello lineare tarato per piccoli errori. Il vantaggio della radice sta nella convergenza finale, non nella reazione ai grandi scostamenti. Nella spinta da 4,5 N·s $s_1$ arriva a $-7{,}6$ rad/s **[S-G]**, dove il guadagno equivalente è 9 s⁻¹.

**Parametri.**

| Costante | Valore | Ruolo e origine |
|---|---|---|
| $c_1$ | 10 s⁻¹ | pendenza della superficie; il commento (righe 62-65) riporta che sul modello linearizzato l'anello regge fino a $c_1 = 40$ anche con rumore e ritardo **[E]**, e indica come limite reale la risonanza delle gambe a 12,5 Hz **[E]** |
| $k_a$ | 25 | tempo di raggiungimento: con il solo termine in radice, da $s_1(0)$ si arriva a zero in $t = 2\sqrt{\lvert s_1(0)\rvert}/k_a$, cioè 0,08 s da $s_1 = 1$ **[D]** |
| $k_b$ | 250 rad/s³ | termine integrale; $L < 250$ (vedi sopra) |
| $\varepsilon$ | 0,02 rad/s | strato limite |
| $W_{max}$ | 60 rad/s² | limite di sicurezza sull'integratore; equivale a 0,6 N·m per ruota in posa nominale ($60/(2\cdot50{,}15)$). Lo storico riporta che oltre 2 m/s $W$ si incolla a ±60 e il beccheggio diverge **[E]** |
| $\Delta t_{max}$ | 4 ms | un passo in ritardo non integra $k_b$ su tutto l'intervallo |

**Saturazione e aderenza.** Il comando comune è limitato per ruota a
$$|\tau_c|\le\min\big(10,\ 0{,}8\,\mu\,N_{ruota}\,r\big) - |\tau_d|,\qquad N_{ruota} = \max\big(\tfrac12 F_n,\ 0{,}35\,m_bg\big).$$
Il limite è la coppia che la ruota può trasmettere senza slittare, con un margine del 20%. Con la forza di contatto di Gazebo, che è costante (§2.4), vale 1,83 N·m. È circa cinque volte meno dei 10 N·m dell'attuatore: il commento a riga 715 dice correttamente che è l'aderenza, non il motore, a limitare. Nelle spinte eseguite per questo documento il limite non è mai stato raggiunto: la coppia massima dello SMC è 0,95 N·m con 2,7 N·s e 1,38 N·m con 4,5 N·s **[S-G]**. L'MPC non ha questo limite e con 4,5 N·s chiede fino a 3,73 N·m per ruota **[S-G]**, cioè più di quanto l'aderenza nominale consenta ($\mu N r = 2{,}78$ N·m con il peso totale).

**Protezione dell'integratore.** $W$ viene congelato quando $\tau_c$ satura, azzerato in volo e azzerato a ogni cambio di fase (`_set_phase`, riga 379), insieme all'integrale dell'anello esterno. Porta infatti la memoria di una condizione di contatto diversa, e trascinarla nella fase successiva produrrebbe un gradino di coppia.

### 5.4 Imbardata

$$\tau_d = \operatorname{sat}_{2,5}\Big(-0{,}89\,e_\varphi - 0{,}21\,(\dot\varphi - \dot\varphi_{ref}) + \text{feedforward attrito} + \text{integrale}\Big)$$
Il feedforward d'attrito, l'integrale con zona morta e la saturazione dell'errore sono identici all'MPC (§4.5).

**I guadagni PD sono la riga d'imbardata del TV-LQR, arrotondata**: l'LQR dà $0{,}894$ e $0{,}214$ **[C]**. Con $\ddot\varphi = 2b_3\tau_d$ i poli sono $-7{,}33\pm2{,}90j$, contro $-7{,}45\pm2{,}62j$ dell'LQR; lo scarto viene dall'arrotondamento di 0,2136 a 0,21 **[C]**. La scelta di **non** usare lo sliding mode qui è motivata dalla stessa risonanza torsionale citata per l'LQR (§4.4) **[E]**: un termine commutante sul modo differenziale la eccita in un ciclo limite.

### 5.5 Gambe: sliding mode in spazio operativo

Per ciascuna gamba (`_leg_smc`, riga 484), nella terna `base_link`:
$$s_{leg} = (v_f - v_d) + c\,(p_f - p_d),\qquad F = -K_{lin}\,s_{leg} - K_{sw}\,\operatorname{sat}\!\big(s_{leg}/\phi\big) + F_{ff},\qquad \tau = J^TF + b_{leg}\dot q .$$

**Equivalenza con il VMC.** Senza il termine commutante:
$$-K_{lin}\big[\dot e + c\,e\big] = -K_{lin}c\,e - K_{lin}\dot e,$$
cioè un VMC con $K_p = K_{lin}c$ e $K_d = K_{lin}$. I parametri riproducono esattamente i guadagni del VMC già validati **[C]**:

| Fase | $c$ [s⁻¹] | $K_{lin}$ [N·s/m] | $K_p = K_{lin}c$ [N/m] | VMC dell'MPC |
|---|---|---|---|---|
| HOLD | 37,5; 40 | 40; 50 | 1500; 2000 | 1500; 2000 |
| Appoggio | 37,5; **48** | 40; **25** | 1500; **1200** | 1500; **0** |
| Volo | 40; 40 | 20; 20 | 800; 800 | 800; 800 |

**L'unica differenza reale è la rigidezza verticale in appoggio**: 1200 N/m con 25 N·s/m, contro 0 e 15 dell'MPC. Nell'MPC la quota è regolata dall'MPC stesso tramite $F_z$. Qui $F_z = m_b(g + \ddot z_{ref})$ è un feedforward in anello aperto, e senza molla verticale la quota non avrebbe alcuna retroazione (`docs/Storico_lavoro.md` §2.2). Ordine di grandezza con $m_b/2$ per gamba: $\omega_n = 26$ rad/s e $\zeta = 0{,}27$ **[C]**. Il valore 25 N·s/m smorza più dei 15 dell'MPC, ma l'anello verticale resta sottosmorzato.

**Il termine robusto e lo strato limite.** $K_{sw} = 8$ N corrisponde a circa 0,74 N·m al ginocchio ($8/10{,}9$, §1.2). Fuori dallo strato limite ($|s_{leg}| > \phi = 0{,}03$ m/s) aggiunge una forza costante di 8 N che si oppone a $s_{leg}$: è il termine che il VMC non aveva e che, nell'ipotesi sliding mode, respinge perturbazioni di forza limitate. **Dentro** lo strato limite il termine è lineare con guadagno $K_{sw}/\phi = 267$ N·s/m e si somma a $K_{lin}$ **[C]**:

| | $K_d$ effettivo [N·s/m] | $K_p$ effettivo [N/m] |
|---|---|---|
| appoggio, $\lvert s_{leg}\rvert > \phi$ | 40; 25 | 1500; 1200 |
| appoggio, $\lvert s_{leg}\rvert < \phi$ | 307; 292 | 11500; 14000 |

Per piccoli scostamenti la gamba dello SMC è quindi circa dieci volte più rigida e smorzata del VMC di cui riproduce i guadagni. L'equivalenza con il VMC vale solo per errori grandi. È una proprietà strutturale della saturazione con strato limite, non un errore. Va però tenuta presente quando si confrontano i due controllori sulle gambe.

### 5.6 Differenze nel salto

Stessa macchina a stati e stessi profili di quota dell'MPC (§4.8). Cambia la forza verticale: nello SMC è $F_z = \operatorname{sat}\big(m_b(g+\ddot z_{ref})\big)$ fra $0{,}3\,m_bg$ e $\{3; 6\}\,m_bg$, senza ottimizzazione. L'inseguimento della quota durante spinta e atterraggio è affidato alla molla verticale da 1200 N/m (o 14000 N/m dentro lo strato limite) e al termine robusto.

---

## 6. Disturbi: perché SMC e MPC rispondono allo stesso modo

Lo sliding mode è stato scelto per la sua fama nella reiezione dei disturbi. Alla prova di spinta, però, SMC e MPC reagiscono quasi allo stesso modo. Questa sezione stabilisce **perché** succede e **se è giusto** che succeda.

### 6.1 L'osservazione, ripetuta in Gazebo

Spinta orizzontale all'indietro sul torso, 4 s dopo il rilascio, applicata come forza costante per 25 passi di fisica (12,5 ms). Campagna `rotino_benchmark`, 18 s per prova **[S-G]**:

| Metrica | MPC 2,7 N·s | SMC 2,7 N·s | MPC 4,5 N·s | SMC 4,5 N·s |
|---|---|---|---|---|
| Picco di beccheggio in avanti (recupero) | **11,36°** | **12,09°** | **11,75°** | **12,23°** |
| Oscillazione iniziale all'indietro (primi 0,4 s) | −0,59° | −6,45° | −0,94° | −12,68° |
| Velocità di picco [m/s] | 0,98 | 1,11 | 1,92 | 2,69 |
| Spazio percorso [m] | 0,37 | 0,56 | 0,91 | 1,49 |
| Coppia di modo comune massima [N·m] | 3,22 | 0,95 | 3,66 | 1,37 |
| Recupero entro ±0,5° [s] | 2,22 | 8,72 | 2,45 | 10,70 |
| Beccheggio rms a regime | 0,001° | 0,141° | 0,004° | 0,137° |
| Chattering, $\lvert\Delta\tau\rvert$ medio per campione [N·m] | 0,0000 | 0,0254 | 0,0000 | 0,0250 |

Il picco in avanti, cioè la grandezza che di solito si guarda, differisce di meno di 1°. Su tutto il resto lo SMC non è migliore:
- nei primi 100 ms lascia ruotare il pendolo all'indietro molto di più (fino a −12,7° contro −0,9°);
- percorre più strada e impiega più tempo;
- a regime ha un'attività di coppia che l'MPC non ha.

Il tempo di recupero dello SMC è gonfiato dal suo rumore a regime (0,14° rms contro una banda di ±0,5°), ma anche guardando le traiettorie il ritorno è più lento (§6.5). Il comparatore del benchmark riporta come "picco" il massimo in valore assoluto, che per lo SMC con 4,5 N·s è l'oscillazione all'indietro (12,68°): per questo la tabella li separa.

### 6.2 Che cosa promette davvero lo sliding mode

La robustezza dello sliding mode è una proprietà precisa, con tre condizioni:

1. **Vale sulla superficie.** Una volta raggiunta $s = 0$, il moto è invariante rispetto ai disturbi che rispettano le ipotesi (Utkin). Durante la **fase di raggiungimento** non c'è invarianza: il sistema si comporta come un qualsiasi controllore non lineare.
2. **Vale per disturbi adattati** (*matched*), cioè che entrano nel sistema attraverso lo stesso canale dell'ingresso di controllo, nel range della matrice $B$. Per il super-twisting la condizione è sulla derivata: $|\dot\rho|\le L$.
3. **Vale per disturbi limitati** entro la classe per cui i guadagni sono dimensionati ($k_b > L$).

La spinta viola tutte e tre le condizioni, per ragioni fisiche e non di taratura.

### 6.3 Anatomia di una spinta

**È un impulso.** La forza agisce per 12,5 ms: un ottavo della costante di tempo del polo instabile (97 ms) e meno di un decimo di qualunque costante di tempo ad anello chiuso. Il suo effetto è un salto quasi istantaneo delle velocità. Dal VL-WIP, con la forza applicata alla `base_link`, a quota $z_b$ sopra l'asse, le forze generalizzate sono $Q_s = F$ e $Q_\theta = F z_b\cos\theta$, e la variazione di velocità è $\Delta\dot\phi = M^{-1}[J,\ Jz_b]^T$ **[D]+[C]**:

| Impulso $J$ | $\Delta\dot s$ | $\Delta\dot\theta$ | $\ddot\theta$ medio durante la spinta |
|---|---|---|---|
| 2,7 N·s | −0,046 m/s | **−4,24 rad/s** | −339 rad/s² |
| 4,5 N·s | −0,077 m/s | **−7,07 rad/s** | −565 rad/s² |

La spinta, applicata sopra il baricentro, fa soprattutto **ruotare** il pendolo all'indietro. La traslazione diretta è piccola.

**La superficie viene abbandonata.** $s_1 = \dot\theta + c_1(\theta-\theta^*)$ salta di circa $\Delta\dot\theta$. In Gazebo si misura $s_1 = -5{,}0$ rad/s con 2,7 N·s e $-7{,}6$ rad/s con 4,5 N·s **[S-G]**: 250-380 volte lo strato limite. Lo SMC entra in fase di raggiungimento, dove non ha alcuna garanzia di invarianza (condizione 1).

**Il disturbo non è adattato, nemmeno per l'anello di beccheggio.** Rispetto al sottosistema $\theta$ la forza entra nello stesso canale di $b_2u$, ma entra anche in $\ddot s$, dove l'ingresso produce $b_1u$ in una proporzione diversa. Per il sistema completo il vettore dei disturbi $[F,\ Fz_b]$ non è parallelo a $B = [1/r,\ -1]$. Il robot è sottoattuato: una coppia di ruota non può annullare contemporaneamente gli effetti su $s$ e su $\theta$ (condizione 2).

**Annullare l'effetto sul beccheggio è fisicamente impossibile.** Per tenere $s_1 = 0$ durante la spinta, la coppia dovrebbe produrre $b_2(\tau_l+\tau_r) = +339$ rad/s², cioè **3,38 N·m per ruota** con 2,7 N·s e **5,64 N·m** con 4,5 N·s **[C]**. L'aderenza nominale è $\mu Nr = 2{,}78$ N·m per ruota con il peso totale, e il limite prudenziale dello SMC è 1,83 N·m. Nessun controllore, sliding mode o no, può rigettare la spinta sul beccheggio senza far slittare le ruote. La derivata della perturbazione, infine, è un fronte di centinaia di rad/s² in meno di un passo di controllo, fuori da qualunque $L$ compatibile con $k_b = 250$ (condizione 3).

**Conseguenza.** Per entrambi i controllori la spinta equivale a una **condizione iniziale**: $\dot\theta\approx-4$ o $-7$ rad/s, con il robot che comincia a muoversi all'indietro. Da lì il recupero è un problema di **regolazione da stato iniziale sotto vincoli**, in cui la reiezione dei disturbi non gioca più alcun ruolo.

### 6.4 Il picco lo decide la saturazione del riferimento di inclinazione

Per fermare un moto all'indietro il robot deve inclinarsi in avanti (§1.6), e per recuperare in fretta deve inclinarsi il più possibile. Entrambi i controllori limitano l'inclinazione **per progetto**:

| | Limite | Angolo | Decelerazione massima a regime |
|---|---|---|---|
| MPC | $\lvert\Delta s\rvert\le$ `DS_MAX_CAP` = 0,03 m | $\operatorname{atan}(0{,}03/0{,}162) = 10{,}48°$ | 1,13 m/s² |
| SMC | $\lvert\theta^*\rvert\le$ `TH_MAX` = 0,21 rad | 12,03° | 1,31 m/s² |

Registrando `/rotino/wbr_state` durante le spinte **[S-G]**:
- **MPC**: $\theta_{ref}$ salta a **+10,48°** al primo aggiornamento dell'MPC dopo la spinta e ci resta 0,9 s (2,7 N·s) o 1,6 s (4,5 N·s). $\theta$ lo raggiunge e sul plateau lo supera al più di 1,1°.
- **SMC**: $\theta^*$ arriva a **+12,03°** in 30-40 ms e ci resta. Con 4,5 N·s $\theta$ vi si appoggia con un plateau fra 12,0° e 12,2° che dura 1,2 s. Sul plateau lo scarto da $\theta^*$ è al più 0,2°.

In entrambi i casi il picco in avanti è **il limite di inclinazione del progetto più un piccolo sorpasso**. La differenza di picco (0,5-0,7°) è più piccola della differenza fra i limiti (1,55°) perché l'MPC supera il proprio limite di circa 1°, mentre lo SMC quasi non lo supera. Il picco in avanti non è quindi un indicatore di reiezione dei disturbi: misura i valori scelti per `DS_MAX_CAP` e `TH_MAX`.

Anche la frenata è simile. La decelerazione media, dal minimo della velocità fino all'arresto, vale 1,33 m/s² (MPC) e 1,28 m/s² (SMC) con 4,5 N·s, e 1,54 m/s² per entrambi con 2,7 N·s **[S-G]**. È dello stesso ordine delle decelerazioni a regime ammesse dai due limiti (1,13 e 1,31 m/s², §1.6). Non coincide con esse perché il tratto misurato comprende il transitorio iniziale e il sorpasso di $\theta$. Una volta che l'inclinazione è saturata, la distanza di arresto è fissata dalla cinematica, circa $v_0^2/(2a_{max})$, e non dalla legge di controllo.

**Dove i due controllori differiscono davvero: i primi 100 ms.** Subito dopo la spinta il pendolo ruota all'indietro con $\dot\theta\approx-4$ o $-7$ rad/s. L'MPC risponde con 3,2-3,7 N·m di modo comune e contiene la rotazione sotto 1°. Lo SMC risponde con 0,95-1,37 N·m e il pendolo arriva a −6,5° e −12,7° **[S-G]**. La coppia dello SMC si ricostruisce dalla legge del §5.3. Con $\dot\theta\approx-4{,}2$ rad/s e $\theta^*$ già a +0,21 rad, $s_1\approx-4{,}2 - 10\cdot0{,}21 = -6{,}3$ rad/s. Allora $\nu\approx -c_1\dot\theta + k_a\sqrt{6{,}3} = 42 + 63 = 105$ rad/s² e $\tau_c\approx 105/(2\cdot(-50)) = -1{,}05$ N·m **[D]**, in accordo con quanto misurato. Il guadagno equivalente del termine in radice, a quell'errore, è circa 10 s⁻¹ (§5.3): per errori grandi il super-twisting è **più morbido** di un anello lineare. È l'unico tratto della risposta in cui la legge sliding mode conta, e va a svantaggio dello SMC.

**Riprova sul modello ridotto.** Sul VL-WIP non lineare, un TV-LQR **senza** l'MPC, quindi senza limite su $\theta_{ref}$, ha il picco che cresce con l'impulso: 10,8° con 2,7 N·s e 17,9° con 4,5 N·s. Lo SMC resta a 12,0° e 13,8° **[S-R]**. Senza il tetto dell'MPC l'LQR si inclinerebbe di più, con coppie fino a 9,8 N·m per ruota, più di tre volte l'aderenza disponibile.

### 6.5 Il recupero lo decidono gli anelli esterni lineari

Dopo il plateau il ritorno alla posizione è governato:
- nell'MPC, dall'MPC stesso e dalla coppia lenta dell'LQR $-1{,}08\pm0{,}68j$;
- nello SMC, dall'**anello esterno PI**, che è lineare e ha un polo a $-0{,}31$ rad/s (§5.2).

Lo sliding mode dello SMC agisce solo sull'anello interno di beccheggio. Una volta passato il transitorio iniziale, in entrambi i controllori quell'anello insegue bene il riferimento: sul plateau lo scarto è al più 1,1° per l'MPC e 0,2° per lo SMC **[S-G]**. La parte del sistema che determina il recupero, cioè come l'inclinazione viene usata per riportare il robot in posizione, è lineare in entrambi. Lo SMC è anche più lento, per scelta di banda.

### 6.6 Dove lo SMC dovrebbe distinguersi: disturbi persistenti

Il vantaggio atteso dell'architettura SMC riguarda i disturbi **persistenti**, che una legge proporzionale-derivativa trasforma in errori a regime. Simulazione sul modello ridotto, disturbo applicato a $t = 1$ s e osservato per 9 s **[S-R]**:

| Disturbo | Grandezza | TV-LQR | SMC |
|---|---|---|---|
| Forza costante −3 N sul torso | $s$ a regime | **−0,654 m** | **−0,023 m** (in convergenza) |
| | $\theta$ a regime | +7,41° | +7,36° |
| Bias di coppia +0,3 N·m per ruota (adattato) | $s$ a regime | **−0,077 m** | **0,000 m** |
| | picco di $\theta$ | −0,70° | −2,57° |
| Momento costante 0,35 N·m sul pendolo (baricentro spostato di circa 1 cm) | $s$ a regime | **+0,325 m** | **+0,011 m** |
| | $\theta$ a regime | −3,56° | −3,51° |

Con una forza o un momento costante l'inclinazione a regime è la stessa per i due controllori, ed è quella richiesta dalla fisica: per stare fermo contro 3 N il robot **deve** inclinarsi di 7,4°. La differenza sta nella **posizione**. Il TV-LQR non ha azione integrale e si assesta lontano dal riferimento; lo SMC riporta il robot in posizione.

Il merito, però, non è della commutazione. Lo SMC annulla l'errore di posizione grazie a due integratori:
- $W$, che nel super-twisting stima e compensa la perturbazione costante sul beccheggio;
- l'integrale $\int e_v$ dell'anello esterno. Con $e_v = \dot s + K_{pos}s$, un valore finito di $\int e_v$ a regime impone $s\to 0$.

Un LQR con azione integrale sulla posizione (LQI) o un MPC con modello del disturbo otterrebbero lo stesso risultato. Il bias adattato mostra anche il prezzo: lo SMC ha un transitorio di beccheggio più ampio (2,57° contro 0,70°) perché il suo anello esterno lento lascia derivare la velocità prima di correggere.

**Sensibilità all'errore di modello** (spinta di 2,7 N·s con $m_b$ reale diverso da quello del modello, **[S-R]**):

| $m_b$ reale / modello | picco $\theta$ LQR | picco $\theta$ SMC | $\lvert s\rvert_{max}$ LQR | $\lvert s\rvert_{max}$ SMC |
|---|---|---|---|---|
| 0,7 | 14,4° | 12,1° | 0,44 m | 0,73 m |
| 1,0 | 10,8° | 12,0° | 0,31 m | 0,42 m |
| 1,3 | 8,6° | 11,6° | 0,24 m | 0,28 m |

Il picco di beccheggio dello SMC è quasi insensibile alla massa, perché è inchiodato a `TH_MAX`, mentre quello dell'LQR varia di ±30%. È l'unico indicatore in cui lo SMC mostra una robustezza parametrica maggiore, e anche qui il meccanismo principale è la saturazione di $\theta^*$, non l'invarianza sulla superficie.

### 6.7 Il costo: chattering a circa 10 Hz

A regime lo SMC ha un'attività di coppia che l'MPC non ha: 0,056 N·m rms con una frequenza dominante fra 9 e 10 Hz in Gazebo, contro 0,0003 N·m dell'MPC **[S-G]**. $|s_1|$ supera lo strato limite il 75% del tempo (rms 0,074 rad/s, 3,7 volte $\varepsilon$) **[S-G]**, quindi il controllore lavora per lo più nella zona in cui $\sigma_\varepsilon$ si comporta come un segno.

**Causa, verificata sul modello ridotto** **[S-R]**. Dopo una piccola perturbazione (0,3 N·s), lo SMC entra in un **ciclo limite autosostenuto a 10,8 Hz** con 0,027 N·m rms. L'LQR, nelle stesse condizioni, si assesta a $2\cdot10^{-6}$ N·m. Togliendo **solo** il filtro su $\dot\theta$ ($\alpha$ da 0,8 a 0), il ciclo scende a $5\cdot10^{-5}$ N·m. Il chattering nasce quindi dall'interazione fra la commutazione quasi discontinua del super-twisting e il ritardo di 9 ms del filtro su $\dot\theta$ (32° di fase a 70 rad/s, §1.8). È il meccanismo classico del chattering da dinamiche non modellate in serie alla superficie. Il modello ridotto non contiene le gambe, quindi la risonanza torsionale a 12,5 Hz non è necessaria per spiegare il fenomeno; in Gazebo potrebbe contribuire, ma non l'ho isolata.

### 6.8 Verdetto: è giusto che rispondano allo stesso modo?

**Sì, e non è un difetto di implementazione.** Le ragioni, in ordine di peso:

1. **La prova misura la cosa sbagliata.** Una spinta di 12,5 ms è un impulso: per qualunque controllore diventa una condizione iniziale. L'invarianza dello sliding mode non si applica in fase di raggiungimento, e la spinta porta $s_1$ a centinaia di volte lo strato limite.
2. **Nessun controllore può rigettare quella spinta.** Tenere fermo il beccheggio richiederebbe 3,4-5,6 N·m per ruota contro un'aderenza di circa 2,8 N·m. Il limite è fisico.
3. **Il picco è fissato dalla saturazione dell'inclinazione di riferimento**, che entrambi i progetti impongono e che ha valori vicini (10,48° e 12,03°). Stesso vincolo, stesso picco.
4. **Il recupero è affidato ad anelli lineari in entrambi.** Nello SMC la parte sliding mode stabilizza il beccheggio, che anche l'LQR stabilizza bene. La traslazione è comandata da un PI lineare più lento della controparte LQR.
5. **La banda dell'anello di beccheggio è simile**: superficie a $-10$ s⁻¹ nello SMC, polo a $-8{,}1$ nell'LQR. A parità di banda e di saturazione, la risposta a una condizione iniziale è simile qualunque sia la legge che la realizza.
6. **L'unica fase in cui la legge sliding mode fa differenza, i primi 100 ms, va a svantaggio dello SMC.** Il termine in radice ha un guadagno equivalente che cala con l'errore, per cui lo SMC contrasta la rotazione iniziale con circa un terzo della coppia dell'MPC e lascia oscillare il pendolo di 6-13° all'indietro invece di meno di 1°.

**Dove l'aspettativa era sbagliata.** "Lo sliding mode rigetta meglio i disturbi" è vero per disturbi **adattati, limitati e persistenti**, a superficie raggiunta. In un pendolo inverso su ruote sottoattuato i disturbi che contano (spinte, pendenze, carichi decentrati) non sono adattati rispetto al sistema completo, e la parte non adattata è gestita da un anello esterno che nello SMC è lineare. Quanto al vantaggio che lo SMC mostra sui disturbi persistenti (§6.6), viene dall'azione integrale, non dalla natura sliding mode della legge.

**Come rendere visibile la differenza.** Le prove che distinguono le due architetture sono quelle con disturbi persistenti o con errori di modello. Nessuna di queste è oggi implementata nel banco di prova:
- una forza orizzontale costante sul torso, applicata per secondi e non per millisecondi, cioè una pendenza equivalente;
- una massa aggiuntiva decentrata sul torso, cioè un momento costante sul pendolo;
- un bias di coppia su una o entrambe le ruote;
- un errore volontario su $m_b$ o su $I_y$ nel modello usato dal controllore.

Su queste prove il modello ridotto prevede errore di posizione a regime per l'MPC e nessun errore per lo SMC. Per non attribuire il risultato allo sliding mode, il confronto andrebbe fatto anche con un TV-LQR con azione integrale.

**Come migliorare lo SMC, se l'obiettivo è la risposta alle spinte:**
- aggiungere un termine lineare proporzionale su $s_1$ (per esempio $-k_ls_1$ accanto al termine in radice), così la reazione ai grandi scostamenti non è più debole di quella di un LQR. È il super-twisting "generalizzato" di Moreno, che conserva la convergenza in tempo finito aggiungendo termini lineari;
- alzare `TH_MAX` entro la validità del modello, per aumentare la decelerazione disponibile. Va fatto con attenzione: l'inversione esatta usa il modello linearizzato;
- alzare la banda dell'anello esterno, oggi 3,5 volte più lento dell'LQR sul modo di posizione;
- ridurre il chattering riducendo il ritardo su $\dot\theta$, per esempio usando il giroscopio più la derivata della cinematica delle gambe invece della differenza finita filtrata, oppure allargando $\varepsilon$.

---

## 7. Sintesi comparativa

| | PID | MPC + TV-LQR + VMC | SMC |
|---|---|---|---|
| Stato per il bilanciamento | ground truth, derivate non filtrate | Kalman + cinematica | Kalman + cinematica |
| Legge sulle ruote | retroazione statica su 4 stati, guadagni per fase | LQR schedulato su $l$ | PI esterno + super-twisting con inversione del modello |
| Uso del modello | nessuno (tarato a mano) | LQR (VL-WIP) + MPC (massa concentrata) | inversione di $a_2, b_2$; $g_{st}$ per la banda esterna |
| Traslazione | termini paralleli $k_x$, $k_{\dot x}$ | MPC con anteprima + LQR sul piano | PI lineare con integrale |
| Limite di inclinazione | nessuno esplicito (saturazione coppia 6,3 N·m) | $\Delta s\le$ 3 cm → 10,48° | $\theta^*\le$ 12,03° |
| Imbardata | PD solo in modalità planare | LQR ($R_d = 100$) + attrito + integrale | PD = riga LQR + attrito + integrale |
| Gambe | PD di giunto, rigidezza verticale ~28 kN/m | VMC, $K_{p,z} = 0$, sostegno da $F_z$ | SMC task-space, 1200 N/m (14 kN/m nello strato limite) |
| Azione integrale | no | solo sull'imbardata | beccheggio ($W$), velocità, imbardata |
| Reazione iniziale a 4,5 N·s **[S-G]** | n/m | −0,9° all'indietro, 3,7 N·m | −12,7° all'indietro, 1,4 N·m |
| Picco in avanti a 4,5 N·s **[S-G]** | n/m | 11,75° (limite 10,48°) | 12,23° (limite 12,03°) |
| Poli sagittali (posa nominale) | $-0{,}149\pm2{,}14j$ (poco smorzati) | $-8{,}13$; $-1{,}08\pm0{,}68j$ | superficie $-10$; esterno $-0{,}76\pm1{,}33j$, $-0{,}31$ |
| Velocità massima (teleop) | n/d | 0,6 m/s | 2,0 m/s |
| Stato **[E]** | in piedi a circa 11° (problema aperto) | θ ≈ 0,00° | ±0,4°, 0,5 ms di CPU per ciclo |

---

## Appendice A. Linearizzazione del VL-WIP passo per passo

Dalle eq. 4-5, riga sagittale, con $u = \tau_l+\tau_r$ e approssimazioni al primo ordine:
$$\underbrace{\begin{bmatrix} m_b+2m_w+2I_w/r^2 & m_bl\\ m_bl & m_bl^2+I_y\end{bmatrix}}_{M_0}\begin{bmatrix}\ddot s\\ \ddot\theta\end{bmatrix} = \begin{bmatrix}u/r\\ -u + m_bgl\,\theta\end{bmatrix}.$$
Posto $M_0 = \begin{bmatrix}\alpha&\beta\\\beta&\gamma\end{bmatrix}$, $\det M_0 = \alpha\gamma - \beta^2$, si ha
$$\ddot s = \frac{\gamma\,u/r - \beta(-u+m_bgl\theta)}{\det M_0},\qquad \ddot\theta = \frac{-\beta\,u/r + \alpha(-u+m_bgl\theta)}{\det M_0}.$$
Moltiplicando numeratore e denominatore per $r^2$: $r^2\det M_0 = \Delta$ del §1.5, e
$$a_1 = -\frac{\beta\,m_bgl\,r^2}{\Delta} = -\frac{g\,l^2m_b^2r^2}{\Delta},\qquad a_2 = \frac{\alpha\,m_bgl\,r^2}{\Delta} = \frac{glm_b\big(2I_w + (m_b+2m_w)r^2\big)}{\Delta}$$
$$b_1 = \frac{(\gamma/r+\beta)\,r^2}{\Delta} = \frac{r\big(I_y + lm_b(l+r)\big)}{\Delta},\qquad b_2 = -\frac{(\beta/r+\alpha)\,r^2}{\Delta} = -\frac{2I_w + r\big(lm_b+(m_b+2m_w)r\big)}{\Delta}.$$
Sono le espressioni dell'eq. 14 e di `vlwip_coefficients`. Imbardata: $M_{33}\ddot\varphi = \frac{d}{2r}(\tau_l-\tau_r)$, da cui $b_3 = \frac{d/(2r)}{M_{33}} = \frac{dr}{2I_zr^2 + d^2(I_w+m_wr^2)}$.

## Appendice B. Costanti principali e loro origine

| Costante | Valore | File:riga | Origine |
|---|---|---|---|
| `LQR_Q` | diag(30, 400, 80, 15, 6, 2) | `rotino_mpc/controller.py:41` | taratura; lettura alla Bryson §4.4 |
| `LQR_R_COMMON` / `_DIFF` | 2 / 100 | `rotino_mpc/controller.py:45-46` | $R = I$ sul comune; differenziale sotto la risonanza **[E]** |
| `WHEEL_TORQUE_MAX` | 10 N·m | MPC :49, SMC :66 | margine sui 18 N·m dell'URDF |
| `DIFF_TORQUE_MAX` | 2,5 N·m | MPC :50, SMC :76 | priorità al bilanciamento |
| `YAW_FRICTION` / `VISCOUS` | 0,25 N·m / 0,10 N·m·s | MPC :53-54, SMC :79-80 | identificati in Gazebo **[E]** |
| `YAW_KI`, `YAW_I_MAX`, `YAW_I_DEADBAND` | 0,6; 0,6; 0,035 | MPC :56-58, SMC :82-84 | integrale lento, anti stick-slip |
| `THETA_DOT_FILTER`, `S_DOT_FILTER` | 0,8 (τ = 9 ms) | MPC :60-61, SMC :86-87 | causa del chattering SMC §6.7 |
| `MPC_HORIZON`, `MPC_DT` | 25, 0,02 s | `rotino_mpc/controller.py:66-67` | orizzonte 0,5 s ≈ 4 $\sqrt{h/g}$ |
| `MPC_S_H`, `MPC_W_H` | (50, 20), 200 | :68-69 | 1 cm di Δs ≈ 2 cm di errore |
| `MPC_S_V`, `MPC_W_V` | (5000, 150), 2e-3 | :70-71 | quota rigida (nessuna molla verticale) |
| `DS_MAX_CAP` | 0,03 m | :72 | **vincolo sempre attivo**, θ ≤ 10,48° |
| `F_MIN/MAX(_THRUST)_RATIO` | 0,3 / 3 / 6 × $m_bg$ | :73-75 | contatto garantito; spinta |
| `VMC_KP/KD` (appoggio) | (1500, 0) / (40, 15) | :81-82 | quota affidata a $F_z$ |
| `KF_Q_ACC`, `KF_R_POS`, `KF_R_VEL` | 0,5; 1e-4; 1e-3 | MPC :89-91, SMC :112-114 | τ di fusione 45 / 280 ms |
| `SMC_KP`, `SMC_KI` | −0,30, −0,10 | `rotino_smc/controller.py:49-50` | banda 1,84 rad/s; segno §5.2 |
| `TH_MAX` | 0,21 rad | :51 | validità del linearizzato; θ ≤ 12,03° |
| `K_POS`, `V_REF_MAX` | 1,2 s⁻¹, 2 m/s | :52-53 | |
| `SMC_C1`, `SMC_KA`, `SMC_KB`, `SMC_EPS1` | 10, 25, 250, 0,02 | :56-59 | §5.3 |
| `SMC_W_MAX`, `SMC_DT_MAX` | 60, 4 ms | :60-61 | protezioni dell'integratore |
| `MU_SAFE`, `N_WHEEL_MIN_RATIO` | 0,8, 0,35 | :67-68 | limite di aderenza 1,83 N·m |
| `YAW_KP`, `YAW_KD` | 0,89, 0,21 | :74-75 | riga d'imbardata dell'LQR arrotondata |
| `LEG_C_*`, `LEG_KLIN_*` | v. §5.5 | :100-105 | riproducono il VMC |
| `LEG_KSW`, `LEG_PHI` | 8 N, 0,03 m/s | :106-107 | termine robusto; 267 N·s/m nello strato |
| `PITCH_OFFSET` | 4,73° | `rotino_pid/controller.py:69` | equilibrio statico (4,694° calcolato) |
| `K_THETA`, `K_THETA_D`, `K_X_D`, `K_X` | 1,3; 0,17; 0,16; 0,8 (× 18 N·m) | :71-74 | tarati in MuJoCo con gambe servo a 2 kHz |
| `MAX_WHEEL` | 0,35 (6,3 N·m) | :70 | |
| `HIP/KNEE_SERVO_KP/KV` | 160/200, 12/14 | :44-50 | servo di posizione dell'originale |
| `LEG_DAMPING_MAX` | 8 N·m | :57 | diagnosi del bang-bang a 500 Hz **[E]** |
| `K_PSI`, `K_OMEGA`, `K_LAT` | 0,10; 0,025; 2,0 | :82-84 | poli di imbardata −26,7, −4,7 |

## Appendice C. Riproducibilità

Script in `docs/verifiche/`:

| File | Cosa produce |
|---|---|
| `numeri.py` | tutti i numeri **[C]**: coefficienti lungo la griglia, zeri, $g_{st}$, $K$ e poli di LQR, PID e imbardata, Kalman a regime, equivalenze delle gambe, profilo di salto, effetto della spinta, verifica simbolica dell'eq. 14 |
| `dist_sim.py` | simulazioni **[S-R]** del §6.4, §6.6 e dei parametri del modello ridotto |
| `chattering_sim.py` | ciclo limite dello SMC con e senza filtro su $\dot\theta$ (§6.7) |
| `record_push.sh`, `rec.py` | lancia un controllore con la spinta e registra `/rotino/wbr_state` (§6.4) |

Esecuzione dalla radice del workspace, con l'ambiente ROS caricato: `python3 docs/verifiche/numeri.py`. Le campagne **[S-G]** si ripetono con

```
ros2 run rotino_benchmark campaign -- --scenario push_enable:=true push_impulse:=2.7 --controllers mpc,smc --duration 18
```

Nota: su questa macchina `scipy` è incompatibile con NumPy 2.2; gli script non lo usano.
