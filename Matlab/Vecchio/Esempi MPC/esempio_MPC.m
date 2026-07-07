%% 1. Definizione del Sistema Massa-Molla-Smorzatore
m = 1;      % Massa [kg]
c = 0.5;    % Coefficiente di smorzamento [Ns/m]
k = 2;      % Costante elastica [N/m]

% Matrici in Spazio di Stato Continuo
A = [0 1; -k/m -c/m];
B = [0; 1/m];
C = [1 0];  % Misuriamo solo la posizione
D = 0;

sys = ss(A, B, C, D);
Ts = 0.1;   % Tempo di campionamento
sys_d = c2d(sys, Ts); % Discretizzazione del sistema

%% 2. Progetto LQR (Senza Vincoli)
% LQR penalizza l'errore di stato (Q) e lo sforzo di controllo (R)
Q = [10 0; 0 1]; % Alta penalità sull'errore di posizione
R = 1;           % Penalità normale sull'uso della forza
K_lqr = dlqr(sys_d.A, sys_d.B, Q, R); 
% Nota: L'LQR restituisce un guadagno fisso u = -K*x. Fine.

%% 3. Progetto MPC (Con Vincoli Espliciti)
PredictionHorizon = 20; % L'MPC "guarda avanti" di 20 passi (2 secondi)
ControlHorizon = 5;     % Ottimizza le mosse per i prossimi 5 passi

% Creazione oggetto MPC
mpc_obj = mpc(sys_d, Ts, PredictionHorizon, ControlHorizon);

% Pesi per la funzione di costo (simili a Q e R dell'LQR)
mpc_obj.Weights.OutputVariables = 10;
mpc_obj.Weights.ManipulatedVariables = 1;

%% 4. Aggiunta dei Vincoli (La vera potenza dell'MPC)

% A. Vincolo sull'ingresso (Manipulated Variable - MV)
% Il nostro motore non può spingere con una forza infinita.
mpc_obj.MV.Min = -3; % Forza minima [N]
mpc_obj.MV.Max = 10;  % Forza massima [N]

% B. Vincolo particolare: Slew Rate (Tasso di variazione)
% I motori si rompono se la forza cambia troppo bruscamente.
% Limitiamo la variazione di forza tra un istante e l'altro.
mpc_obj.MV.RateMin = -0.5; 
mpc_obj.MV.RateMax = 0.5;  

% C. Vincolo sullo stato (Output Variable - OV)
% Mettiamo un "muro" virtuale: la massa non deve mai superare la posizione 1.5m
mpc_obj.OV.Max = 1.5; 

%% 5. Simulazione passo-passo (Custom Loop) per estrarre la Funzione Obiettivo
disp('Simulazione MPC step-by-step verso x = 2...');
T_sim = 50; % Numero di passi
riferimento = 2;

% Vettori per salvare i dati storici per i plot
y_history = zeros(1, T_sim);
v_history = zeros(1, T_sim);
u_history = zeros(1, T_sim);
cost_history = zeros(1, T_sim);

% Inizializzazione dello stato dell'impianto reale (partiamo da fermi in 0)
x_plant = [0; 0]; 
y_misurata = C * x_plant;

% Inizializzazione dello stato interno del controllore MPC
mpc_state = mpcstate(mpc_obj);

options = mpcmoveopt;
% Ciclo di simulazione in tempo reale
for k = 1:T_sim
    % Abbassiamo dinamicamente il limite di forza nel tempo
    options.MVMax =3 + 10* exp(-0.05 * k);
    % 1. L'MPC calcola la mossa ottima basandosi sulla misura attuale
    % 'Info' contiene i segreti del risolutore, incluso il Costo!
    [u, Info] = mpcmove(mpc_obj, mpc_state, y_misurata, riferimento, [], options);
    
    % 2. Salviamo tutti i dati di questo istante k
    y_history(k) = y_misurata;
    v_history(k) = x_plant(2);
    u_history(k) = u;
    cost_history(k) = Info.Cost; % ECCO LA FUNZIONE OBIETTIVO!
    
    % 3. Applichiamo l'ingresso 'u' all'impianto reale (Avanzamento temporale)
    x_plant = sys_d.A * x_plant + sys_d.B * u;
    y_misurata = sys_d.C * x_plant; % Nuova lettura del sensore
    
end

% Vettore del tempo per l'asse X
t = (0:T_sim-1) * Ts;

%% 6. Creazione della Dashboard Grafica
figure('Name', 'Analisi Completa MPC', 'NumberTitle', 'off', 'Position', [100, 100, 900, 700]);

% --- PLOT 1: POSIZIONE ---
subplot(2, 2, 1);
plot(t, y_history, 'b-', 'LineWidth', 2); hold on;
yline(riferimento, 'g--', 'Rif. (2m)', 'LineWidth', 1.5);
yline(mpc_obj.OV.Max, 'r-', 'Muro (1.5m)', 'LineWidth', 2);
grid on; title('Posizione (Stato 1)'); ylabel('[m]'); ylim([0 2.2]);

% --- PLOT 2: VELOCITÀ ---
subplot(2, 2, 2);
plot(t, v_history, 'm-', 'LineWidth', 2);
grid on; title('Velocità (Stato 2)'); ylabel('[m/s]');

% --- PLOT 3: FORZA (INGRESSO) ---
subplot(2, 2, 3);
stairs(t, u_history, 'k-', 'LineWidth', 2); hold on;
yline(mpc_obj.MV.Max, 'r-', 'Max (+3N)', 'LineWidth', 1.5);
yline(mpc_obj.MV.Min, 'r-', 'Min (-3N)', 'LineWidth', 1.5);
grid on; title('Forza Applicata (Ingresso)'); xlabel('Tempo [s]'); ylabel('[N]'); ylim([-4 4]);

% --- PLOT 4: FUNZIONE OBIETTIVO (IL COSTO J) ---
subplot(2, 2, 4);
plot(t, cost_history, 'c-', 'LineWidth', 2);
grid on; title('Evoluzione Funzione Obiettivo (Costo)'); xlabel('Tempo [s]'); ylabel('Valore del Costo J');

sgtitle('Dashboard MPC: Dinamica, Vincoli e Ottimizzazione');