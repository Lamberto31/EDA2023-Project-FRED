% Simulazione FRED

%% RESET INIZIALE
clear;
close all;


%% IMPOSTAZIONI
% True per simulare robot con posizione fissata (posizione indipendente da
% velocità)
fixedPosition = false;

% True per simulare robot con posizione iniziale molto vicina all'obiettivo
nearPosition = false;
nearDistance = 10; %[cm]

% Scelta dello stato da utilizzare per la scelta dell'input
    % 0: x stimato, come si farebbe nello scenario reale;
    % 1: x misurato, come si farebbe senza filtraggio;
    % 2: x reale, come si farebbe nel caso ideale.
x_for_input = 0;

% True per far attendere il robot prima di iniziare a muoversi
waitBeforeStart = false;
if not(waitBeforeStart)
    wait_time = 0; %[s]
else
    wait_time = 0.5; %[s]
end

% Controllo valore corretto per x_for_input
if x_for_input ~= 0 && x_for_input ~= 1 && x_for_input ~= 2
    error("Valore di x_for_input non valido");
end


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
% Valori massimi distanza stop e slow (solo per grafico)
d_stop_slow = v_slow * M/b; %[cm] Distanza per fermarsi da velocità slow
d_stop_fast = v_fast * M/b; %[cm] Distanza per fermarsi da velocità fast
t_vm_max = M/b*log((1/epsilon)*abs(v_fast - v_slow)); %[s] Tempo per arrivare a velocità slow da fast
d_maxSpeed_max = (v_fast - v_slow)*M/b*(exp(-(b/M)*t_vm_max)-1)+ v_slow*t_vm_max; %[cm] Distanza per arrivare a velocità slow da fast
d_slow_max = d_maxSpeed_max + d_stop_fast; %[cm] Distanza per arrivare a velocità slow da fast e fermarsi in tempo

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


%% INIZIALIZZAZIONE MODELLO
% Condizione iniziale reale
if not(nearPosition)
    x(:,1) = [202 0]'; %[cm cm/s]
else
    x(:,1) = [obj+nearDistance 0]'; % %[cm cm/s]
end
% Covarianza stima iniziale
sigma_0 = [66 v_max/100]; %[cm cm/s]
P0 = diag(sigma_0.^2);
% Stima iniziale
% (in media valore reale distribuita gaussianamente con varianza sigma_0
x_hat(:,1) = x(:,1) + diag(sigma_0) * randn(n,1);

% Covarianza stima iniziale
P = P0;

% Input iniziale
if x_hat(1,1) > obj
    u_sign = -1;
else 
    u_sign = 1;
end

u_0 = u_sign * C_fast * 0;


%% SIMULAZIONE (reale simulato e filtrato)
% Secondi da considerare
sec = 10;
K = round(sec/T);
% Pre-allocazioni varie (performance)
z = zeros(p, K);
u = zeros(1, K);
u(:,1) = u_0;
zn = zeros(p, K);
x_pred = zeros(n, K);
W_tot = zeros(1, K);
innovation = zeros(p, K);
error = zeros(n, K);
measurement_error = zeros(p, K);
P1 = zeros(1, 2*K);
P2 = zeros(1, 2*K);
P1(1:2) = P(1,1);
P2(1:2) = P(2,2);
d_stop = zeros(1,K);
d_slow = zeros(1,K);
diff = zeros(1,K);

% Per controllare incidente
crash = false;

% Per controllare input
slowMode = false;
stopMode = false;

f = waitbar(0, "Inizio simulazione");
% Loop principale
for k = 1:K
    % MODELLO REALE
    % Evoluzione
    % Stato reale
    x(:,k+1) = F * x(:,k) + G * u(:,k) + L * sqrt(Q) * randn(n,1);
    % Output reale
    z(:,k+1) = H * x(:,k+1);
    % Output misurato (rumoroso)
    zn(:,k+1) = H * x(:,k+1) + sqrt(R) * randn(p,1);

    % Controllo scontro con ostacolo
    if x(1,k+1) <= 0
        x(:,k+1) = 0;
        crash = true;
    end
    
    
    % STIMA CON FILTRO DI KALMAN
    % Predictor
    % Predizione: x(k+1|k)
    x_pred(:,k+1) = F * x_hat(:,k) + G * u(:, k);
    % Covarianza predizione: P(k+1|k)
    P_pred = F * P * F' + Q; % P[k+1|k]
    
    % Corrector
    % Guadagno
    W = P_pred*H'/(H*P_pred*H'+R);
    % Correzione: x(k+1|k+1)
    x_hat(:,k+1) = x_pred(:,k+1) + W*(zn(:,k+1)-H*x_pred(:,k+1));
    % Covarianza correzione: P(k+1|k+1) (formulazione 2)
    P = (eye(n)-W*H)*P_pred;
    
    % Innovazione
    innovation(:,k+1) = zn(:,k+1)-H*x_pred(:,k+1);
    
    % Varianza innovazione
    S = H * P_pred * H' + R;
    
    % Errore di stima
    error(:,k+1) = x(:,k+1)-x_hat(:,k+1);
    
    % Errore di misura
    measurement_error(:,k+1) = z(:,k+1) - zn(:,k+1);

    % Indici per covarianza
    index_pred = k*2+1;
    index_est = index_pred + 1;
    
    % Covarianza errore stima posizione
    P1(index_pred) = P_pred(1,1);
    P1(index_est) = P(1,1);
    
    % Covarianza errore stima velocità
    P2(index_pred) = P_pred(2,2);
    P2(index_est) = P(2,2);

    % Controllo incidente e uscita
    if crash
        disp("Crash!");
        waitbar(1, f, "Terminato prima per incidente!");
        K = k;
        break;
    end

    % Input
    % Scelta stato per input
    if x_for_input == 0
        x_check = x_hat(:,k+1);
    elseif x_for_input == 1
        x_check = zn(:,k+1);
    else
        x_check = x(:,k+1);
    end

    if (k+1)*T > wait_time && not(stopMode)
        % Calcolo x_stop e x_slow
        d_stop(k+1) = abs(x_check(2))*M/b;
        if not (slowMode)
            t_vm = M/b*log((1/epsilon)*abs(abs(x_check(2)) - v_slow));
            d_maxSpeed = (abs(x_check(2)) - v_slow)*M/b*(exp(-(b/M)*t_vm)-1)+ v_slow*t_vm;
            d_slow(k+1) = d_maxSpeed + d_stop(k+1);
        end
    
        % Check posizione rispetto a x_slow e x_stop
        diff(k+1) = abs(x_check(1) - obj);
        if not(slowMode) && diff(k+1) > d_slow(k+1) 
            u(:,k+1) = u_sign * C_fast;
        elseif not(stopMode) && diff(k+1) > d_stop(k+1)
            if not(slowMode)
                slowMode = true;
                x_slowMode = x_check(1);
                k_slow = k+2;
                disp(" ");
                disp("Input lento")
                disp("Passo: " + string(k+1));
                disp("Istante: " + string((k+1)*T) + " s");
                disp("Posizione reale: " + string(x(1,k+1)));
                disp("Posizione stimata: " + string(x_hat(1,k+1)));
                disp("Velocità reale: " + string(x(2,k+1)));
                disp("Velocità stimata: " + string(x_hat(2,k+1)));
                disp("Velocità veloce massima: " + string(v_fast));
            end
            u(:,k+1) = u_sign * C_slow;
        else
            if not(stopMode)
                stopMode = true;
                x_stopMode = x_check(1);
                k_stop = k+2;
                disp(" ");
                disp("Input nullo")
                disp("Passo: " + string(k+1));
                disp("Istante: " + string((k+1)*T) + " s");
                disp("Posizione reale: " + string(x(1,k+1)));
                disp("Posizione stimata: " + string(x_hat(1,k+1)));
                disp("Velocità reale: " + string(x(2,k+1)));
                disp("Velocità stimata: " + string(x_hat(2,k+1)));
                disp("Velocità bassa massima: " + string(v_slow));
            end
            u(:,k+1) = 0;
        end
    end
    
    % Aggiornamento waitbar
    waitbar(k/K, f, "Simulazione in corso");
end
pause(1);
close(f)


%% RISULTATI
disp(" ");
disp("Distanza obiettivo: " + string(obj));
disp("Posizione finale reale: " + string(x(1,end)));
disp("Posizione finale stimata: " + string(x_hat(1,end)));
disp("Errore posizione reale: " + string(abs(obj - x(1,end))));
disp("Errore posizione stimato: " + string(abs(obj - x_hat(1,end))));
disp("Velocità finale reale: " + string(x(2,end)));
disp("Velocità finale stimata: " + string(x_hat(2,end)));


%% GRAFICI
timeStepString = "time step ["+ string(T) + " s]";
% Input
figure;
plot(u(1,:)); hold on;
xlabel(timeStepString);
ylabel('input');
title('Input');

% Posizione
figure;
plot(zn(1,:),'r'); hold on; 
plot(x_hat(1,:),'b');
plot(x(1,:),'g');   
plot(ones(1,K)*obj);
plot(ones(1,K)*x_slowMode);
plot(ones(1,K)*x_stopMode);
legend('measurement','estimate','real', 'objective','slow position','stop position');
xlabel(timeStepString);
ylabel('position [cm]');
title('Position');

% Velocità
figure;
plot(zn(2,:), 'r'); hold on;
plot(x_hat(2,:),'b');
plot(x(2,:),'g');
plot(-ones(1,K)*v_fast);
plot(-ones(1,K)*v_slow);
legend('measurement','estimate','real','max fast','max slow');
xlabel(timeStepString);
ylabel('velocity [cm/s]');
title('Velocity');

% Mappa robot-ostacolo
figure;
%set(gca, 'XAxisLocation', 'origin'); 
plot(0, 0, 'o'); hold on;
plot(x(1,1), 0, '+');
plot(x(1,2:end), zeros(K,1), 'g');
plot(x(1,end), 0, 'x');
plot(obj, 0, '*');
axis equal;
grid on;
legend('Obstacle', 'Start poisition', 'Trajectory', 'Final position', 'Objective');
xlabel('Position (cm)')
title('Tracjectory');

% Distanze per rallentare/fermarsi
figure;
plot(d_stop(1:k_stop)); hold on;
plot(d_slow(1:k_stop));
plot(d_stop_fast*ones(1,k_stop));
plot(d_stop_slow*ones(1,k_stop));
plot(d_slow_max*ones(1,k_stop));
k_diff_start = max(1,k_slow-10);
plot(k_diff_start:k_stop, diff(k_diff_start:k_stop));
legend('d_{stop}','d_{slow}','d_{stop_{fast}}','d_{stop_{slow}}','d_{slow_{max}}','estimate - objective');
xlabel(timeStepString);
ylabel('Distance [cm]');
title('Distance tresholds');

% Covarianza errore di stima posizione
first_index = max(2*wait_time/T,1);
figure;
plot(P1(first_index:end)); hold on;
xlabel(timeStepString);
ylabel('position estimation error covariance [cm^2]');
legend('P(1,1)')
title('Position estimation error covariance');

% Covarianza errore di stima velocità
figure;
plot(P2(first_index:end)); hold on;
xlabel(timeStepString);
ylabel('velocity estimation error covariance [(cm/s)^2]');
legend('P(2,2)')
title('Velocity estimation error covariance');

% Errore stima posizione
figure;
plot(error(1,:)); hold on;
plot(measurement_error(1,:),'--');
legend('estimation error','measurement error');
xlabel(timeStepString);
ylabel('position [cm]');
title('Position error');