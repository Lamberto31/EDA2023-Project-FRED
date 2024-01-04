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


%% INIZIALIZZAZIONE MODELLO
% Condizione iniziale reale
x(:,1) = [202 0]'; %[cm cm/s]
% Covarianza stima iniziale
sigma_0 = [66 v_max/100]; %[cm cm/s]
P0 = diag(sigma_0.^2);
% Stima iniziale
% (in media valore reale distribuita gaussianamente con varianza sigma_0
x_hat(:,1) = x(:,1) + diag(sigma_0) * randn(n,1);

% Covarianza stima iniziale
P = P0;

% Input iniziale
u_0 = C_fast;


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
    
    % Covarianza errore stima posizione
    P1(1+(k-1)*2) = P_pred(1,1);
    P1(1+k*2) = P(1,1);
    
    % Covarianza errore stima velocità
    P2(1+(k-1)*2) = P_pred(2,2);
    P2(1+k*2) = P(2,2);

    % Controllo incidente e uscita
    if crash
        disp("Crash!");
        waitbar(1, f, "Terminato prima per incidente!");
        K = k;
        break;
    end

    % Input
    if not(stopMode)
        % Calcolo x_stop e x_slow
        x_stop = abs(x_hat(2,k))*M/b + obj;
        t_vm = M/b*log((1/epsilon)*abs(abs(x_hat(2,k)) - v_slow));
        x_slow = (abs(x_hat(2,k)) - v_slow)*M/b*(exp(-(b/M)*t_vm)-1)+ v_slow*t_vm + x_stop;
    
        % Check posizione rispetto a x_slow e x_stop
        if not(slowMode) && x_hat(1,k) > x_slow 
            u(:,k+1) = - C_fast;
        elseif not(stopMode) && x_hat(1,k) > x_stop
            if not(slowMode)
                slowMode = true;
                x_slowMode = x_hat(1,k);
                disp("Input lento")
                disp("Posizione reale: " + string(x(1,k)));
                disp("Posizione stimata: " + string(x_hat(1,k)));
                disp("Velocità reale: " + string(x(2,k)));
                disp("Velocità stimata: " + string(x_hat(2,k)));
                disp("Velocità veloce massima: " + string(v_fast));
            end
            u(:,k+1) = - C_slow;
        else
            if not(stopMode)
                stopMode = true;
                x_stopMode = x_hat(1,k);
                disp("Input nullo")
                disp("Posizione reale: " + string(x(1,k)));
                disp("Posizione stimata: " + string(x_hat(1,k)));
                disp("Velocità reale: " + string(x(2,k)));
                disp("Velocità stimata: " + string(x_hat(2,k)));
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
xlabel('time step');
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
xlabel('time step');
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
xlabel('time step');
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


% Covarianza errore di stima posizione
figure;
plot(P1); hold on;
xlabel('time step');
ylabel('position estimation error covariance [cm^2]');
legend('P(1,1)')
title('Position estimation error covariance');

% Covarianza errore di stima velocità
figure;
plot(P2); hold on;
xlabel('time step');
ylabel('velocity estimation error covariance [(cm/s)^2]');
legend('P(2,2)')
title('Velocity estimation error covariance');

% Errore stima posizione
figure;
plot(error(1,:)); hold on;
plot(measurement_error(1,:),'--');
legend('estimation error','measurement error');
xlabel('time step');
ylabel('position [cm]');
title('Position error');