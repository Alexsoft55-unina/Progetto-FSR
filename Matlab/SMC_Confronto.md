# Confronto tra le due varianti SMC (`SMC/` vs `SMC_Aldo/`)

Aggiornato al 19/09/2026.

Riferimento: Z. Cui et al., *Modeling and Control of a Wheeled Biped Robot*, Micromachines 2022, 13, 747.

Cartelle equivalenti in `Simscape/Robot/`: stesso controllore (`wbr_ctrl.m`), incapsulato in un modello Simscape invece che in dinamica simbolica pura.

---

## ⚠️ Attenzione: la stessa notazione `Q` indica giunti diversi

Entrambe le varianti usano `Q = [q1; qw; q2; q3]`, ma **q1 e q2 non sono lo stesso giunto fisico**:

| Indice | `Matlab/SMC` (principale) | `Matlab/SMC_Aldo` |
|---|---|---|
| `Q(1)` = q1 | **Stinco/caviglia**, angolo assoluto — **PASSIVO** | **Ginocchio** — attuato (`tau_1`) |
| `Q(2)` = qw | Ruota — attuato (`tau_w`) | Ruota — attuato (`tau_w`) |
| `Q(3)` = q2 | **Ginocchio**, angolo relativo (coscia = q1+q2) — attuato | **Anca** — attuato (`tau_2`) |
| `Q(4)` = q3 | **Torso**, angolo assoluto — attuato | **Torso**, angolo assoluto — **NON attuato**, evolve per dinamica libera |

`SMC_Aldo/init.m` dichiara esplicitamente di seguire la "convenzione ESATTA del paper (Cui et al., 2022, Table 1)"; `Matlab/SMC` usa invece una propria convenzione con la caviglia come grado passivo.

**Rischio concreto:** riutilizzare `robot_params`, `K.q1_ref`/`K.q2_ref` o codice fra le due cartelle senza rimappare gli indici scambia il riferimento del ginocchio con quello dello stinco/caviglia. I due controllori **non sono intercambiabili** a livello di stato o di guadagni.

---

## Architettura di controllo (comune a entrambe)

Cascata a due velocità di tempo:

1. **Anello esterno (lento):** PI sull'errore di velocità del CoM totale → genera il riferimento di pitch `θ_b*`. Guadagni **negativi** (`K.KP`, `K.KI` < 0) perché il sistema è a fase non minima. Saturazione `±K.TH_MAX` non negoziabile, con anti-windup sull'integratore.
2. **Anello interno (veloce), canale pitch:** super-twisting SMC su `θ_b`, con stato integrale `W` da integrare esternamente (blocco Simulink: Unit Delay/Integrator).
3. **Postura:** SMC classico + boundary layer sui due giunti di gamba rimanenti.

## Differenze nella legge di controllo

| Aspetto | `Matlab/SMC` | `Matlab/SMC_Aldo` |
|---|---|---|
| Giunti di postura controllati | ginocchio (q2) + **torso** (q3) | ginocchio (q1) + **anca** (q2) |
| Sostituto continuo del segno | `tanh(s/eps)` ovunque | `s/(|s|+eps)` per il super-twisting; `sat_(x)=min(max(x,-1),1)` per gli altri canali |
| Legge canali 2-3 | `-c·d - k·s - η·tanh(s/φ)` | `-k·sat(s/φ) - η·s` (termine lineare aggiuntivo oltre al boundary layer) |
| Disaccoppiamento | Partial Feedback Linearization non collocata: complemento di Schur (`D`, `Mbar`, `hbar` da `wbr_terms.m`), poi `A = [gth'; 0 1 0; 0 0 1]` | Inversione diretta via matrice di ingresso `B` esplicita: `Ad = Jy*(M\B)`, `tau = Ad \ (...)` |
| Anti-windup / freeze | Solo sull'integratore esterno `Iv` | Anche il **super-twisting si congela** (`dW=0`) quando la coppia satura |
| Diagnostica | `cond(A)` (condizionamento disaccoppiamento) | `σ = M(2,2)+M(4,2)` (perdita di autorità della ruota) |
| Firma funzione | `wbr_ctrl(Q, dQ, v_ref, Iv, W, K)` | `wbr_ctrl(Q, dQ, v_ref, Iv, W, p, K)` — richiede anche i parametri fisici `p` |

## Generazione della dinamica

- **`Matlab/SMC`**: script `wbr_build_model.m` (il file è salvato come `init.m` nella cartella) deriva Lagrangiana con angoli assoluti `q1` (stinco), genera `calc_dynamics.m` + `calc_kin.m` (uscite, Jacobiane e termini di deriva in un solo file) e include un ciclo diagnostico che stampa `cond(A)` su una griglia di posture.
- **`Matlab/SMC_Aldo`**: `init.m` deriva la stessa Lagrangiana ma con angoli assoluti costruiti a partire da ginocchio/anca/torso (coerente con la Tabella 1 del paper), genera `calc_dynamics.m` e **separatamente** `calc_com_jacobian.m` (Jacobiano del CoM rispetto a q1, q2, per il VMC). La cinematica di uscita (`th`, `J_th`, ecc.) è scritta a mano in `wbr_kin.m`, non generata.

## File di supporto specifici

- `Matlab/SMC/Claude/` — variante generata con Claude, con una sua sotto-copia `sIM/` (stessi nomi di file di `Matlab/SMC/`, verificarne l'allineamento prima di considerarla la versione corrente).
- `Matlab/SMC/scooter_simulation.mp4`, `sim.pdf` — output di una simulazione già eseguita.
- `Simscape/Robot/SMC/Relazione_Controllo_WBR.tex` e `Relazione_Dinamica_Controllo_WBR.tex` — relazioni LaTeX con la derivazione completa; i soli sorgenti `.tex` e il `.pdf` compilato sono tracciati in git, i file di build (`.aux/.log/.out/.synctex.gz`) sono stati rimossi e ora ignorati.

## Pulizia effettuata (19/09/2026)

Rimossi dal repository perché ridondanti/rigenerabili:
- `Simscape/Robot/SMC_Aldo.zip`, `SMC_Aldo (2).zip`, `SMC_Aldo (3).zip` — backup incrementali della cartella `SMC_Aldo/` già presente estratta.
- `Matlab/SMC/untitled.slx.autosave` — autosave di Simulink.
- File di build LaTeX in `Simscape/Robot/SMC/` (`.aux`, `.log`, `.out`, `.synctex.gz`).

Aggiunte a `.gitignore`: `*.slx.autosave`, `*.aux`, `*.log`, `*.out`, `*.synctex.gz`.
