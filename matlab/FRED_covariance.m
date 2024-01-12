% Simulazione Covarianza FRED

%% RESET INIZIALE
clear;
close all;


%% IMPOSTAZIONI
% True per simulare robot con posizione fissata (posizione indipendente da
% velocità)
fixedPosition = false;


%% DEFINIZIONE DATI
% PARAMETRI
% Caratteristiche robot
% Massa robot (sperimentale)
M = 0.731; %[kg]
% Tensione di alimentazione (picco) (sperimentale)
Vp = 6; %[V]
% Velocità di regime (ingresso costante massimo) (sperimentale)
v_max = 70; %[cm/s]
% Tempo di arresto da velocità massima (ingresso costante nullo)
% (sperimentale)
t_0 = 0.362; %[s]
% Diametro ruota (datasheet)
D = 6.5; %[cm]
% Impulsi per giro (buchi encoder) (visibile)
IPR = 20; %[pulse/round] %IPR = pi*D;

% Parametri derivati
% Coefficiente attrito motore
b = 5*(M/t_0); %[kg/s]
% Fattore di proporzionalità forza-tensione
eta_V = v_max*(b/Vp); %[N/V *10^-2] (perchè cm invece di metri)
% Costante comoda per calcoli
kappa = eta_V*Vp/255; %[N *10^-2] (perchè cm invece di metri)


% MATRICI
% Passo di discretizzazione
T= 0.01; %[s]
% Termini che appaiono spesso (per comodità)
bmT = T*(1 - ((b/M) * (T/2)));
% F
F = [1 bmT;
    0 1-(b/M)*bmT];
% G
G = ((1/M) * eta_V * (Vp/255)) * [T^2/2;
    bmT];
% H
H = [1 0;
    0 IPR/(pi*D)];
% LAMBDA
L = [T (1/2)*T^2;
    0 bmT];

% Se il robot è sospeso e non cambia posizione
if fixedPosition
    F(1,2) = 0; %#ok<UNRCH>
    G(1) = 0;
    L(1,2) = 0;
end

% Dimensioni vettori stato e misura
% Stato
n = size(F,1);
% Misura
p = size(H,1);

% INCERTEZZA
% Processo
sigma_qp = 0.03;
% sigma_qp = 0;
sigma_qv = 0.01;
Q = diag([ sigma_qp^2 sigma_qv^2]);
% Misura
sigma_p = 0.3; %[cm]
sigma_v = 0.1; %[pulse/round]
R = diag([ sigma_p^2 sigma_v^2]);


%% OSSERVABILITA'
% Rango della matrice di osservabilità
r = rank(obsv(F,H));

% Il sistema è osservabile?
if r == n
    disp("Sistema osservabile")
else
    disp("Sistema non osservabile")
end


%% INIZIALIZZAZIONE MODELLO
% Covarianza stima iniziale
sigma_0 = [66 v_max/100]; %[cm cm/s]
P0 = diag(sigma_0.^2);
P = P0;

% Controllo se simmetrica e semidefinita positiva
[P0_symm, P0_semidefpos] = checkCovariance(P);

%% SIMULAZIONE (reale simulato e filtrato)
% Secondi da considerare
sec = 10;
K = round(sec/T);
% Pre-allocazioni varie (performance)
P1 = zeros(1, 2*K);
P2 = zeros(1, 2*K);
P1(1:2) = P(1,1);
P2(1:2) = P(2,2);
P_symm = zeros(1, 2*K);
P_semdefpos = zeros(1, 2*K);
P_symm(1:2) = P0_symm;
P_semdefpos(1:2) = P0_semidefpos;
% Variabile per capire se finito prima
interrupted = false;

f = waitbar(0, "Inizio simulazione");
% Loop principale
for k = 1:K
    % CALCOLI
    % Covarianza predizione: P(k+1|k)
    P_pred = F * P * F' + Q; % P[k+1|k]
    
    % Corrector
    % Guadagno
    W = P_pred*H'/(H*P_pred*H'+R);
    % Covarianza correzione: P(k+1|k+1) (formulazione 2)
    P = (eye(n)-W*H)*P_pred;
    
    % Varianza innovazione
    S = H * P_pred * H' + R;

    % Indici per covarianza
    index_pred = k*2+1;
    index_est = index_pred + 1;
    
    % Covarianza errore stima posizione
    P1(index_pred) = P_pred(1,1);
    P1(index_est) = P(1,1);
    
    % Covarianza errore stima velocità
    P2(index_pred) = P_pred(2,2);
    P2(index_est) = P(2,2);

    % Check covarianza
    [P_symm(index_pred), P_semdefpos(index_pred)] = checkCovariance(P_pred);
    [P_symm(index_est), P_semdefpos(index_est)] = checkCovariance(P);
    
    % Interruzione simulazione se covarianza errata
    P_pred_condition = [P_symm(index_pred), P_semdefpos(index_pred)];
    P_est_condition = [P_symm(index_est), P_semdefpos(index_est)];
    if any(~P_pred_condition)
        interrupted = true;
        P_wrong = "pred";
        P_condition = P_pred_condition;
    elseif any(~P_est_condition)
        interrupted = true;
        P_wrong = "est";
        P_condition = P_est_condition;
    end

    if interrupted
        % Indicazione matrice errata
        if P_wrong == "pred"
            disp("Covarianza predizione (P_pred) errata!")
            disp(P_pred)
        elseif P_wrong == "est"
            disp("Covarianza stima (P_est) errata!")
            disp(P)
        end
        % Indicazione motivo
        if all(~P_condition)
            disp("Problema per entrambe le condizioni")
        elseif ~P_condition(1)
            disp("Matrice non simmetrica")
        elseif ~P_condition(2)
            disp("Matrice non semidefinita positiva")
        end
        waitbar(1, f, "Terminato prima per matrice di covarianza errata!")
        K = k;
        break;
    end


    
    % Aggiornamento waitbar
    waitbar(k/K, f, "Simulazione in corso");
end
pause(1);
close(f)


%% RISULTATI
disp(" ");
if interrupted
    disp("Ultimo passo: " + string(K));
end
disp("Covarianza predizione finale:");
disp(P_pred);
disp("Covarianza stima finale:");
disp(P);

%% GRAFICI
timeStepString = "time step ["+ string(T) + " s]";
% Covarianza errore di stima posizione
figure;
plot(P1(1:index_est)); hold on;
xlabel(timeStepString);
ylabel('P(1,1) [cm^2]');
title('Position estimation error covariance');

% Covarianza errore di stima velocità
figure;
plot(P2(1:index_est)); hold on;
xlabel(timeStepString);
ylabel('P(2,2) [(cm/s)^2]');
title('Velocity estimation error covariance');