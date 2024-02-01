% View params FRED

%% RESET INIZIALE
clear;
close all;


%% PARAMS %%
% Log files directory
logsPath = '../raspberry/logs/';

% Name of csv file with extension
csvName = 'FRED_params_2024-02-01T17_24_30_bulk_moving_attempts_10.csv';

% Number of attempts to consider (if you want to use all use intmax)
numAttempts = 10;

% Consider distance (enable the usage of distance to do statistics)
% Useful since the experiment is done in fixed position sometimes
useDistance = true;

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
speed_median = zeros(numAttempts,1);
T_speed_distance = cell(numAttempts,1);
speed_max_distance = zeros(numAttempts,1);
speed_mean_distance = zeros(numAttempts,1);
speed_median_distance = zeros(numAttempts,1);
for i = 1:numAttempts
    % Take only speed when increasing and not 0
    idx_speed_notNull = T_attempts{i}.speed ~= 0 & T_attempts{i}.status == "Input max";
    T_speed{i} = T_attempts{i}(idx_speed_notNull, :);
    % Exclude too low value based on a first mean
    first_mean = mean(T_speed{i}.speed);
    idx_speed_high = T_speed{i}.speed > first_mean*50/100;
    T_speed{i} = T_speed{i}(idx_speed_high, :);
    % Statistics
    speed_mean(i) = mean(T_speed{i}.speed);
    speed_median(i) = median(T_speed{1}.speed);
    speed_max(i) = max(T_speed{i}.speed);
    speed_mean_all = mean(speed_mean);
    speed_median_all = median(speed_median);
    speed_max_all = mean(speed_max);
    
    % Using distance
    if useDistance
        T_speed_distance{i} = T_speed{i};
        % Compute all speed using distance and current time differences
        % Vectorial
        % T_speed_distance{i}.speed(2:end) = diff(T_speed_distance{i}.distance) ./ diff(T_speed_distance{i}.currentTime);
        for j = 1: height(T_speed{i})-1
            deltaT = (T_speed{i}.currentTime(j+1) - T_speed{i}.currentTime(j)) / 1000;
            deltaD = T_speed{i}.distance(j+1) - T_speed{i}.distance(j);
            T_speed_distance{i}.speed(j+1) = abs(deltaD / deltaT);
        end
        % Exclude first value since is not computable
        T_speed_distance{i} = T_speed_distance{i}(2:end,:);
        % Statistics
        speed_mean_distance(i) = mean(T_speed_distance{i}.speed);
        speed_median_distance(i) = median(T_speed_distance{i}.speed);
        speed_max_distance(i) = max(T_speed_distance{i}.speed);
        speed_mean_all_distance = mean(speed_mean_distance);
        speed_median_all_distance = mean(speed_median_distance);
        speed_max_all_distance = mean(speed_max_distance);
    end

end
% Time to Stop
% Take rows that contains a value for tts
idx_timeToStop = not(isnan(T.stopTime));
T_stop = T(idx_timeToStop,:);
% Statistics
tts_mean = sum(T_stop.stopTime) / numAttempts;

%% SHOW RESULTS
% Prepare results
if not(useDistance)
    speed_mean_disp = speed_mean;
    speed_median_disp = speed_median;
    speed_max_disp = speed_max;
    speed_statistics_all_disp = table([speed_mean_all, speed_median_all, speed_max_all]', 'VariableNames', "Speed", 'RowNames', ["Average mean", "Average median", "Average max"]);
else
    speed_mean_disp = table(speed_mean, speed_mean_distance, 'VariableNames', ["Speed", "Distance"]);
    speed_median_disp = table(speed_median, speed_median_distance, 'VariableNames', ["Speed", "Distance"]);
    speed_max_disp = table(speed_max, speed_max_distance, 'VariableNames', ["Speed", "Distance"]);
    speed_statistics_all_disp = table([speed_mean_all, speed_median_all, speed_max_all]', [speed_mean_all_distance, speed_median_all_distance, speed_max_all_distance]', 'VariableNames', ["Speed", "Distance"], 'RowNames', ["Average mean", "Average median", "Average max"]);
end
speed_statistics_disp = table(speed_mean_disp, speed_median_disp, speed_max_disp, 'VariableNames', ["Mean", "Median", "Max"]);

% Speed
disp("MAX SPEED [cm/s]")
disp("Results for each experiment");
disp(speed_statistics_disp);
disp("Average results");
disp(speed_statistics_all_disp);

% Time to Stop
disp("TIME TO STOP [ms]")
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
hplot4(end+1) = plot(T_speed{i}.currentTime, ones(length(T_speed{i}.currentTime),1)*speed_median_all, 'DisplayName', 'Average median');
hplot4(end+1) = plot(T_speed{i}.currentTime, ones(length(T_speed{i}.currentTime),1)*speed_max_all, 'DisplayName', 'Average max');
title('Speed (used for statistics)');
legend(hplot4);
xlabel('Time [ms]');
ylabel('Speed [cm/s]');
grid on

% Speed based on distances (statistics)
if useDistance
    figure();
    hold on;
    hplot5 = zeros(1, numAttempts);
    for i = 1:numAttempts
        hplot5(i) = plot(T_speed_distance{i}.currentTime, T_speed_distance{i}.speed, '-o', 'DisplayName', int2str(i));
    end
    hplot5(end+1) = plot(T_speed_distance{i}.currentTime, ones(length(T_speed_distance{i}.currentTime),1)*speed_mean_all_distance, 'DisplayName', 'Average mean');
    hplot5(end+1) = plot(T_speed_distance{i}.currentTime, ones(length(T_speed_distance{i}.currentTime),1)*speed_median_all_distance, 'DisplayName', 'Average median');
    hplot5(end+1) = plot(T_speed_distance{i}.currentTime, ones(length(T_speed_distance{i}.currentTime),1)*speed_max_all_distance, 'DisplayName', 'Average max');
    title('Speed based on distances (used for statistics)');
    legend(hplot5);
    xlabel('Time [ms]');
    ylabel('Speed [cm/s]');
    grid on
end

% Speed mean and max values (both method)
figure();
% Layout definition
tiledlayout(useDistance+1,1);

% Speed
ax1 = nexttile;
hold on
hplot6 = plot(1:numAttempts, speed_mean, '-+', 'DisplayName', 'Mean');
hplot7 = plot(1:numAttempts, speed_median, '-+', 'DisplayName', 'Median');
hplot8 = plot(1:numAttempts, speed_max, '-+', 'DisplayName', 'Max');
hplot9 = plot(1:numAttempts, ones(numAttempts,1)*speed_mean_all, 'DisplayName', 'Average mean');
hplot10 = plot(1:numAttempts, ones(numAttempts,1)*speed_median_all, 'DisplayName', 'Average median');
hplot11 = plot(1:numAttempts, ones(numAttempts,1)*speed_max_all, 'DisplayName', 'Average max');
title('Speed statistics (based on speed)');
legend([hplot6, hplot7, hplot8, hplot9, hplot10, hplot11]);
xlabel('Attempt');
ylabel('Speed [cm/s]');
grid(ax1,'on')
hold off

% Distance
if useDistance
    ax2 = nexttile;
    hold on
    hplot12 = plot(1:numAttempts, speed_mean_distance, '-+', 'DisplayName', 'Mean');
    hplot13 = plot(1:numAttempts, speed_median_distance, '-+', 'DisplayName', 'Median');
    hplot14 = plot(1:numAttempts, speed_max_distance, '-+', 'DisplayName', 'Max');
    hplot15 = plot(1:numAttempts, ones(numAttempts,1)*speed_mean_all_distance, 'DisplayName', 'Average mean');
    hplot16 = plot(1:numAttempts, ones(numAttempts,1)*speed_median_all_distance, 'DisplayName', 'Average median');
    hplot17 = plot(1:numAttempts, ones(numAttempts,1)*speed_max_all_distance, 'DisplayName', 'Average max');
    title('Speed statistics (based on distance)');
    legend([hplot12, hplot13, hplot14, hplot15, hplot16, hplot17]);
    xlabel('Attempt');
    ylabel('Speed [cm/s]');
    grid(ax2,'on')
    hold off
end