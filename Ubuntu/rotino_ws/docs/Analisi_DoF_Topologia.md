# Analisi di Gradi di Libertà (DoF), Topologia e Sottoattuazione del Wheeled Biped Robot (WBR)

Questa analisi ricalca gli argomenti trattati nelle diapositive del corso (in particolare le slide *Introduction* e *Wheeled Robots*) applicandoli all'architettura specifica del nostro progetto WBR (Wheeled Biped Robot).

---

## 1. Gradi di Libertà (DoF) e Spazio delle Configurazioni ($\mathcal{C}$-space)

Il calcolo dei gradi di libertà si effettua analizzando inizialmente il robot come meccanismo nello spazio, prima di imporre i vincoli di contatto con il suolo.

1.  **Corpo rigido spaziale (Torso base):**
    Il torso del WBR può muoversi liberamente nello spazio tridimensionale. Come definito nel corso, un corpo rigido spaziale possiede **6 DoF** (3 traslazioni $x, y, z$ e 3 rotazioni nel $SO(3)$).
2.  **Giunti interni (Cinematica delle gambe):**
    *   2 giunti d'anca (Hip): di tipo *revolute*, aggiungono **2 DoF**.
    *   2 giunti di ginocchio (Knee): di tipo *revolute*, aggiungono **2 DoF**.
    *   2 giunti delle ruote (Wheel): di tipo *continuous*, aggiungono **2 DoF**.

**Totale DoF non vincolati**:
Applicando la formula di Grübler estesa per robot in volo/spazio, sommiamo i gradi di libertà del base link a quelli dei giunti:
$$\text{DoFs} = 6 \text{ (base)} + 2 \text{ (hip)} + 2 \text{ (knee)} + 2 \text{ (wheel)} = 12 \text{ DoF}$$

Il vettore delle coordinate generalizzate (non vincolato) $\boldsymbol{q} \in \mathbb{R}^{12}$ è:
$$ \boldsymbol{q} = [x, y, z, \phi, \theta, \psi, q_{hl}, q_{hr}, q_{kl}, q_{kr}, q_{wl}, q_{wr}]^T $$

---

## 2. Topologia dello Spazio delle Configurazioni

Come evidenziato nel corso, la topologia ("forma" dello spazio delle configurazioni) è cruciale: spazi con lo stesso numero di DoF non sono necessariamente equivalenti topologicamente (non si possono deformare l'uno nell'altro senza tagliarli).

*   **Torso (Base Link):** Vive nello spazio euclideo speciale $SE(3)$, che è topologicamente equivalente a $\mathbb{R}^3 \times SO(3)$. (Per evitare singolarità di rappresentazione, o Gimbal Lock, utilizziamo internamente i quaternioni per $SO(3)$).
*   **Anche e Ginocchia (Limiti di giunto):** Nel nostro robot fisico e nel file URDF, anche e ginocchia hanno limiti fisici e non possono compiere giri completi. Topologicamente, un giunto *revolute* con limiti meccanici non è una varietà circolare $S^1$, ma un intervallo chiuso in $\mathbb{R}$. Per 4 giunti, abbiamo una topologia equivalente a $\mathbb{R}^4$.
*   **Ruote (Senza limiti):** I giunti delle ruote possono ruotare all'infinito. Ognuna vive su una circonferenza $S^1$. Insieme formano un Toro $T^2 = S^1 \times S^1$.

**Topologia complessiva del $\mathcal{C}$-space libero:**
$$ \mathcal{C} = SE(3) \times \mathbb{R}^4 \times T^2 $$

---

## 3. Analisi dei Vincoli: Olonomi e Anolonomi

Quando il robot poggia a terra, lo stato non è più libero di variare arbitrariamente. Nel corso (*Wheeled Robots*), si specifica che i robot su ruote sono soggetti a vincoli geometrici (olonomi) e cinematici (anolonomi, in forma Pfaffiana $A^T(q)\dot{q} = 0$).

1.  **Vincoli Olonomi (Integrabili):**
    Il contatto delle ruote con il terreno piano blocca il moto verticale della base (a meno del piegamento delle gambe) e ne riduce i movimenti di rollio. Nelle equazioni di Lagrange, la forza di reazione vincolare non compare direttamente ma si traduce in una riduzione delle coordinate essenziali del *task space*.
2.  **Vincoli Anolonomi (Non Integrabili):**
    Le ruote impongono una condizione di puro rotolamento (assenza di slittamento laterale e longitudinale). Questo si traduce in vincoli cinematici del tipo $\dot{x} \sin(\psi) - \dot{y} \cos(\psi) = 0$ per la traiettoria sul piano.
    Essendo il sistema cinematico $\dot{q} = G(q)u$ completamente controllabile (le parentesi di Lie dei campi vettoriali riempiono lo spazio tangente, l'involuzione non è chiusa), tali vincoli limitano le direzioni di *spostamento istantaneo* ma non riducono i DoF staticamente raggiungibili dal robot.

Le equazioni dinamiche del WBR diventano quindi (come da slide):
$$ M(q)\ddot{q} + h(q, \dot{q}) = B(q)\boldsymbol{\tau} + A(q)\boldsymbol{\lambda} $$
dove $\boldsymbol{\lambda}$ rappresenta i moltiplicatori di Lagrange delle reazioni vincolari del suolo.

---

## 4. Analisi della Sottoattuazione (Underactuation)

Nel corso l'underactuation è descritta come la condizione in cui i controlli indipendenti sono meno delle variabili di configurazione/task.

### Attuatori disponibili ($m$):
Il WBR dispone di **6 attuatori**:
*   2 motori alle ruote ($\tau_{wl}, \tau_{wr}$)
*   2 motori alle ginocchia ($\tau_{kl}, \tau_{kr}$)
*   2 motori alle anche ($\tau_{hl}, \tau_{hr}$)
Dimensione del controllo: $m = \text{dim}(\boldsymbol{\tau}) = 6$.

### Valutazione della Sottoattuazione:
Come ricavato in precedenza, il robot libero ha $n = 12$ DoF.
Poiché $m < n$ ($6 < 12$), per definizione la matrice di distribuzione dell'attuazione $B(q)$ non ha rango pieno rispetto a tutte le coordinate. Pertanto, **l'intero sistema è formalmente sottoattuato**.

Calando l'analisi sul **Modello Operativo (Task Space)** (citando il professore: *"We refer to underactuation as a property of the mathematical model used to model our robotic system"*), la natura sottoattuata resta evidente e strutturale:

1.  **Coordinate inattuate e Fase Non-Minima:**
    L'obiettivo vitale è bilanciare il *beccheggio (pitch)* del torso $\theta$. Non possediamo un attuatore (es. un'elica) posizionato in alto in grado di generare una coppia assoluta per $\ddot{\theta}$. Per variare $\theta$, dobbiamo comandare una coppia alle ruote che, tramite la reazione vincolare $A(q)\lambda$ del terreno (vincolo anolonomo), genera un'accelerazione longitudinale. Questo significa che usiamo **lo stesso attuatore per controllare due coordinate ($s$ e $\theta$)**. Di conseguenza, per far avanzare il robot, dobbiamo prima sbilanciarlo in avanti decelerando momentaneamente le ruote (comportamento a fase non minima, zeri di metà destra).
2.  **Modello Ridotto e Teorema di Brockett:**
    Delegando il controllo dell'altezza virtuale $Z$ alle articolazioni (VMC), per l'LQR noi mappiamo il robot al modello VL-WIP (Pendolo Inverso a Lunghezza Variabile). Questo sotto-sistema ha $m=1$ (coppia di avanzamento comune) e $n=2$ (gradi di libertà attivi sagittali $s$ e $\theta$).
    Infine, come esplicitamente riportato nel corso per i modelli di tipo *unicycle/differential-drive*, i vincoli anolonomi ricadono sotto il **Teorema di Brockett**: questo implica che il robot non ammette *nessuna* legge di controllo universale tempo-invariante e liscia in grado di stabilizzarne asintoticamente in un punto esatto $(x_d, y_d, \psi_d)$.
