% Simulazione FRED

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


% INPUT
% Distanza desiderata
obj = 10; %[cm]
% Input veloce
C_fast = 255; 
v_fast = kappa*C_fast/b; %[cm/s]
% Input lento
C_slow = 100;
v_slow = kappa*C_slow/b; %[cm/s]
% Tolleranza velocità
epsilon = 0.01; %[cm/s]


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
    x_slowMode = 0;
    x_stopMode = 0;
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