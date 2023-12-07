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
tiledlayout(3,1);

% Position
ax1 = nexttile;
hold on
hplot1 = plot(timestampDate, T.field1, '-o', 'DisplayName', 'US');
hplot2 = plot(timestampDate, T.field2, '-*', 'DisplayName', 'Optical');
hplot3 = plot(timestampDate, T.field3, '-pentagram', 'DisplayName', 'Filtered');
hplot8 = plot(timestampDate, T.field8, '-.', 'DisplayName', 'Objective');
title('Position');
legend([hplot1, hplot2, hplot3, hplot8]);
grid(ax1,'on')
hold off

% Velocity
ax2 = nexttile;
hold on
hplot5 = plot(timestampDate, T.field5, '-o', 'DisplayName', 'US');
hplot6 = plot(timestampDate, T.field6, '-*', 'DisplayName', 'Optical');
hplot7 = plot(timestampDate, T.field7, '-pentagram', 'DisplayName', 'Filtered');
title('Velocity');
legend([hplot5, hplot6, hplot7]);
grid(ax2,'on')
hold off

% Rps
ax3 = nexttile;
hplot4 = plot(timestampDate, T.field4, '-*', 'DisplayName', 'Optical');
title('RPS');
legend(hplot4);
grid(ax3,'on')

