%% PARAMS %%
% Log files directory
logsPath = '../raspberry/logs/';

% Name of csv file with extension
csvName = 'FRED_params_2023-12-07T14_45_55_fixed_wood_attempts_10.csv';

% Number of attempts to consider (if you want to use all use intmax)
numAttempts = 10;

%% GET DATA FROM CSV %%
path = strcat(logsPath,csvName);
opts = detectImportOptions(path);
T = readtable(path, opts);

% Get only first numAttempts
rows = sum(T.attempt <= numAttempts);
T = T(1:rows,:);

% Get the real number of considered attempts (if numAttempts > real)
numAttempts = T(end,:).attempt;

%% DIVIDE ATTEMPTS %%
% Create cell array used to divide attempts
T_attempts = cell(numAttempts,1);
for i = 1:numAttempts
    idx = T.attempt == i;
    T_attempts{i} = T(idx, :);
end

%% STATISTICS %%
% Speed
T_speed = cell(numAttempts,1);
speed_max = zeros(numAttempts,1);
speed_mean = zeros(numAttempts,1);
for i = 1:numAttempts
    % Take only speed when increasing and not 0
    idx_speed_notNull = T_attempts{i}.speed ~= 0 & T_attempts{i}.note == "Increasing";
    T_speed{i} = T_attempts{i}(idx_speed_notNull, :);
    % Statistics
    speed_mean(i) = mean(T_speed{i}.speed);
    speed_max(i) = max(T_speed{i}.speed);
    speed_mean_all = mean(speed_mean);
    speed_max_all = mean(speed_max);
end
% Time to Stop
% Take rows that contains a value for tts
idx_timeToStop = not(isnan(T.stopTime));
T_stop = T(idx_timeToStop,:);
% Statistics
tts_mean = sum(T_stop.stopTime) / numAttempts;

% Show results
% Speed
disp("Mean speed for each experiment [cm/s]");
disp(speed_mean);
disp("Max speed for each experiment [cm/s]");
disp(speed_max);
disp("Mean speed [cm/s");
disp(speed_mean_all);
disp("Mean max speed [cm/s");
disp(speed_max_all)
% Time to Stop
disp("Mean time to stop [ms]");
disp(tts_mean);

%% VISUALIZE DATA %%
% Position and speed
figure();
% Layout definition
tiledlayout(2,1);

% Distance
ax1 = nexttile;
hold on
hplot1 = zeros(1, numAttempts);
for i = 1:numAttempts
    hplot1(i) = plot(T_attempts{i}.currentTime, T_attempts{i}.distance, '-o', 'DisplayName', int2str(i));
end
title('Distance');
legend(hplot1);
xlabel('Time [ms]');
ylabel('Distance [cm]');
grid(ax1,'on')
hold off

% Speed
ax2 = nexttile;
hold on
hplot2 = zeros(1, numAttempts);
for i = 1:numAttempts
    hplot2(i) = plot(T_attempts{i}.currentTime, T_attempts{i}.speed, '-*', 'DisplayName', int2str(i));
end
title('Speed');
legend(hplot2);
xlabel('Time [ms]');
ylabel('Speed [cm/s]');
grid(ax2,'on')
hold off

% Time to stop
if numAttempts > 1
    figure(); hold on;
    plot(1:numAttempts, T_stop.stopTime, '-+');
    plot(1:numAttempts, ones(numAttempts,1)*tts_mean);
    title('Time to stop');
    xlabel('Attempt');
    ylabel('Time to stop [ms]');
    legend("For each attempt", "Mean");
    grid on
end

% Speed (statistics)
figure();
hold on;
hplot4 = zeros(1, numAttempts);
for i = 1:numAttempts
    hplot4(i) = plot(T_speed{i}.currentTime, T_speed{i}.speed, '-*', 'DisplayName', int2str(i));
end
hplot4(end+1) = plot(T_speed{i}.currentTime, ones(length(T_speed{i}.currentTime),1)*speed_mean_all, 'DisplayName', 'Average mean');
hplot4(end+1) = plot(T_speed{i}.currentTime, ones(length(T_speed{i}.currentTime),1)*speed_max_all, 'DisplayName', 'Average max');
title('Speed (statistics)');
legend(hplot4);
xlabel('Time [ms]');
ylabel('Speed [cm/s]');
grid on
