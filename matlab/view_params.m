%% PARAMS %%
% Log files directory
logsPath = '../raspberry/logs/';

% Name of csv file with extension
csvName = 'FRED_log_2023-10-17T16_07_56_objective.csv';

% Number of attempts to consider
numAttempts = 5;

%% GET DATA FROM CSV%%
path = strcat(logsPath,csvName);
opts = detectImportOptions(path);
T = readtable(path, opts);

% Get only first numAttempts
rows = sum(T.attempt <= numAttempts);
T = T(1:rows,:);

% Get the real number of considered attempts (if numAttempts > real)
numAttempts = T(end,:).attempt;

%% DIVIDE ATTEMPTS
% Create cell array used to divide attempts
T_attempts = cell(numAttempts,1);
for i = 1:numAttempts
    idx = T.attempt == i;
    T_attempts{i} = T(idx, :);
end

%% VISUALIZE DATA %%
% Layout definition
tiledlayout(2,1);

% Position
ax1 = nexttile;
hold on
hplot1 = zeros(1, numAttempts);
for i = 1:numAttempts
    hplot1(i) = plot(T_attempts{i}.currentTime, T_attempts{i}.distance, '-o', 'DisplayName', int2str(i));
end
title('Position');
legend(hplot1);
grid(ax1,'on')
hold off

% Velocity
ax2 = nexttile;
hold on
hplot2 = zeros(1, numAttempts);
for i = 1:numAttempts
    hplot2(i) = plot(T_attempts{i}.currentTime, T_attempts{i}.speed, '-*', 'DisplayName', int2str(i));
end
title('Velocity');
legend(hplot2);
grid(ax2,'on')
hold off

