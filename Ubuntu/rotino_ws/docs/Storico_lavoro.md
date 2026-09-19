# Storico del lavoro — robot bipede su ruote

Documento di passaggio di consegne, scritto il 20/09/2026. Serve a ricostruire in fretta **dove sono le cose, cosa è stato deciso e perché, cosa è verificato e cosa no**, senza dover ripercorrere le indagini già fatte.

---

## 1. Mappa: tre posizioni, ruoli diversi

| Percorso | Cos'è | Stato |
|---|---|---|
| `~/Progetto-FSR` | Repo git storico: Matlab, Simscape, paper, tesi, più una **copia** dei workspace ROS sotto `Ubuntu/` | Committato fino a `7e265d1` |
| `~/FSR_robot` | Repo git dei workspace ROS 2 vivi: `PID/`, `MPC/`, `SMC/` | `PID` e `MPC` intatti; `SMC/` non tracciato |
| `~/rotino_ws` | **Workspace unificato nuovo**, robot rinominato RoTino, sei package | Non tracciato da git |

⚠️ `Progetto-FSR/Ubuntu/MPC` è una **copia più vecchia** di `FSR_robot/MPC`, non un link. Le due divergono. La copia in `Progetto-FSR` è servita una volta per recuperare un file non tracciato.

---

## 2. Cosa è stato fatto, in ordine

### 2.1 Pulizia di `Progetto-FSR` (commit `7e265d1`)

Rimossi tre zip di backup ridondanti (~29 MB), un autosave Simulink, i file di build LaTeX. Aggiunte regole `.gitignore`. Creato `Matlab/SMC_Confronto.md`.

**Scoperta non ovvia, ancora valida**: le due varianti Matlab `SMC/` e `SMC_Aldo/` usano la **stessa notazione `Q(1..4)` per giunti fisicamente diversi**. In `SMC/` la caviglia è passiva e si attuano ginocchio+torso; in `SMC_Aldo/` si segue la Tabella 1 di Cui et al. e si attuano ginocchio+anca lasciando il torso libero. Non sono intercambiabili senza rimappare gli indici.

### 2.2 Implementazione SMC in `FSR_robot/SMC`

Cascata sliding mode pura che sostituisce MPC + TV-LQR + VMC:

```
posizione → velocità → θ*   (PI, guadagni NEGATIVI: fase non minima)
θ* → super-twisting su s₁ = θ̇ + c₁(θ−θ*) → coppia comune ruote
imbardata → PD + attrito + integrale → coppia differenziale
gambe → SMC task-space per lato, riferimento comune
```

**Decisione architetturale chiave.** L'ipotesi "un controllo per gamba, replicato per simmetria" vale per **le gambe** ma non per il bilanciamento: il beccheggio del torso è un **unico** grado di libertà condiviso, e due SMC indipendenti si contenderebbero la stessa superficie di scorrimento. Confermato alla fonte leggendo Cui et al. (il PDF è in `FSR_robot/MPC/`), Sez. 3 e 3.3: la simmetria serve a *ridurre a un unico modello sagittale*, non a duplicare il controllore.

**Due fatti che hanno reso il lavoro fattibile:**
- `vlwip_coefficients` dà già la forma control-affine `θ̈ = a₂θ + b₂(τ_l+τ_r)`: l'inversione per il super-twisting è **esatta**, niente nuova modellistica.
- Il VMC era **già** uno sliding mode privo del termine discontinuo: `Kp·e + Kd·ė ≡ −Kd[ė + (Kp/Kd)e]`. I guadagni validati si conservano (verificato a 6·10⁻¹⁴ N).

**Trappola risolta**: in appoggio `VMC_KP = diag(1500, 0)` — la rigidezza verticale era azzerata *di proposito* perché il sostegno veniva da `F_z` dell'MPC. Tolto l'MPC va **ripristinata** (1200 N/m), altrimenti la quota resta senza anello chiuso.

### 2.3 Bug trovato alzando la velocità

Portando il limite a 1 m/s il robot entrava in **ciclo limite permanente** (±6° che non si esauriva). Causa: l'anello esterno usava la velocità del **baricentro**, e `V_com = V_b + ω × R·c_b` contiene la velocità di beccheggio — quando il robot dondola quel termine vale ~0,4 m/s, quanto la marcia vera, e si chiude una retroazione positiva.

**Correzione**: l'anello esterno usa la velocità dell'**asse**, già filtrata. È anche il segnale che il TV-LQR del paper usava nel suo stato.

Limiti di velocità verificati: **2 m/s funziona**, 2,5 e 3 m/s **cadono** (θ\* satura → l'integratore del twisting `W` si incolla a ±60 → il beccheggio scappa). In `rotino_ws` il limite è fissato a 2,0.

### 2.4 Creazione di `~/rotino_ws`

Sei package, robot rinominato `sebaju` → `RoTino` (package, moduli, topic `/rotino/*`, mondo, modello Gazebo, nodi).

```
rotino_description   URDF, mondo, config + libreria di modello CONDIVISA
rotino_pid           PD in cascata
rotino_mpc           MPC + TV-LQR + VMC
rotino_smc           cascata sliding mode
rotino_dashboard     dashboard unificata
rotino_benchmark     logger + campagne + confronto
```

La libreria (`kinematics`, `model`, `planar_trajectory`) sta in **una sola copia** dentro `rotino_description`: prima era duplicata in tre. Se diverge, il confronto misura modelli diversi invece di leggi di controllo — è esattamente quello che è già successo fra `Matlab/SMC` e `Matlab/SMC_Aldo`.

Il PID è stato **portato a coppia** su richiesta, così tutti e tre usano la stessa attuazione. Effetto collaterale utile: tutti e tre pubblicano ora `/rotino/debug` con gli stessi 11 campi e le coppie gambe sullo stesso topic, quindi il logger del benchmark è identico per tutti.

---

## 3. Stato verificato

| Cosa | Esito |
|---|---|
| `rotino_mpc` in Gazebo | ✅ si bilancia, θ ≈ 0,00° |
| `rotino_smc` in Gazebo | ✅ ±0,4°, CPU 0,5 ms su 2 ms |
| `rotino_pid` in Gazebo | ⚠️ **in piedi ma a ~11°**, contro ~5° dell'originale |
| `rotino_benchmark` | compila, **mai eseguito end-to-end** |
| `rotino_dashboard` | compila, slider a 2 m/s |
| `FSR_robot/PID` originale | ✅ funziona ancora, ±3–7° |

Risultati SMC misurati in `FSR_robot/SMC` (headless): bilanciamento ±0,26°; quota inseguita a <1 mm su sinusoide ±3 cm; impulso 2,7 N·s → picco 11,4°, recupero ~3,5 s; 1 m/s su 5 m → arresto a 4,99 m.

---

## 4. Problema aperto: il PID a coppia

**Sintomo originale**: entro 0,5 s dal rilascio il robot veniva scagliato.

**Causa, stabilita con misure**: non i guadagni, non il codice, non l'URDF. È l'**energia iniettabile per ciclo**. Il PD di giunto girava a 2 kHz dentro Gazebo, ora gira a 500 Hz nel nodo:

```
α = τ_max/I = 60/0,007 = 8570 rad/s²
Δv per ciclo a  500 Hz = 17,1 rad/s
Δv per ciclo a 2000 Hz =  4,3 rad/s
```

Quattro cicli bastano per arrivare a 70 rad/s; allora `kv·q̇ = 840 N·m` satura e **inverte segno a ogni campione** → bang-bang.

**Rimedio applicato**: limitare **solo il termine di smorzamento** a ±8 N·m, lasciando piena autorità proporzionale. Da caduta a 11,5° medio.

**Contro-intuitivo ma solido**: abbassare `kv` **peggiora sempre**, fino alla caduta. Lo smorzamento serve; va limitata l'energia, non il guadagno.

### Cause già escluse — non riaprirle senza motivo nuovo

| Sospetto | Come è stato escluso |
|---|---|
| `dt` del ciclo | misurato: 2,000 ms costanti |
| URDF / mondo / config | diff normalizzati: solo rinomina + attuazione + `ApplyLinkWrench` |
| Codice del controllore | diff: la legge PD è **identica**, cambia solo la destinazione |
| Frequenza odometria | identica, 500 Hz in entrambi |
| Controller non attivati | log di spawn: tutti e tre attivi |
| Coppia stantia | `_publish_legs` chiamato ogni ciclo, nessuna uscita anticipata |
| Segno della coppia | invertendolo il robot cede invece di oscillare → il verso è giusto |
| Riscalamento di `kv`, ammorbidimento di `kp` | provati entrambi, il calcio resta |

**Prossimo passo concordato**: ritarare i guadagni di bilanciamento sulle ruote (`k_th`, `k_thd`, `k_x`, `k_xd`), tarati assumendo gambe rigide a 2 kHz. L'utente ha autorizzato la ritaratura.

File di lavoro in `~/rotino_ws/diagnostics/`: i cinque `.diff` normalizzati, `06_diagnosi.txt` con misure e scansione completa, `_orig_normalized.py` (originale con la rinomina applicata, per diff puliti), `_controller_backup.py`.

---

## 5. Trappole incontrate — costano ore se le ripeti

- **`pkill -f <pattern>` uccide la shell chiamante** se il pattern compare nella sua stessa riga di comando. Mi è successo due volte. Usa il trucco delle parentesi: `pkill -f "[i]nstall/rotino"`.
- **Processi orfani falsano le prove.** Una dashboard rimasta viva pubblicava su `/rotino/cmd_vel` e metteva il controllore in teleop: una prova a 3 m/s è risultata completamente invalida. Controlla sempre `grep -c "Live commands received"` nel log.
- **`setup.cfg` scritto via `printf` in bash**: `\$base` produce un backslash letterale e gli eseguibili non vengono installati. Il sintomo è `libexec directory does not exist` al launch.
- **`--symlink-install` sui package Python ROS copia, non collega.** Editare `src/` non basta: serve ricompilare (dura <2 s).
- **MDPI, ScienceDirect e simili rispondono 403** alle richieste automatiche. I PDF vanno scaricati dall'utente e letti da disco. L'utente ha ~25 paper in `Progetto-FSR/Papers/` e `Ricerche/`, molti IEEE — vale la pena guardare lì **prima** di cercare in rete.

---

## 6. Convenzioni da rispettare

- Documentazione e commenti del progetto: **in italiano** per i documenti, **in inglese** per i commenti nel codice (segue lo stile preesistente).
- `/rotino/debug` deve restare a **11 campi** e `/rotino/wbr_state` a **18**: `logger.py` li spacchetta posizionalmente e `ros_bridge.py` valida la lunghezza.
- Il PID **non** pubblica `wbr_state` (solo MPC e SMC). Il substrato comune del confronto è `/rotino/debug`.
- I workspace in `FSR_robot` restano **intatti**: non toccarli senza chiedere.
- Non committare senza richiesta esplicita.
