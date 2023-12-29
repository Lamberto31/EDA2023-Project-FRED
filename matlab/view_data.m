%% PARAMS %%
% Log files directory
logsPath = '../raspberry/logs/';

% Name of csv file with extension
csvName = 'FRED_log_2023-10-17T16_07_56_objective.csv';

% Number of values to show
numPoints = 50;


%% GET DATA FROM CSV%%
path = strcat(logsPath,csvName);
opts = detectImportOptions(path);
T = readtable(path, opts);

% Get the last numPoints rows
T = T(max(1,(end-(numPoints - 1))):end, :);

% Extract timestamp and convert from epoch to datetime
timestampEpoch = T.created_at;
% timestampDate = datetime(timestampEpoch,'ConvertFrom','posixtime','TimeZone','Europe/Zurich','Format','dd-MMM-yyyy HH:mm:ss');
timestampDate = datetime(timestampEpoch,'ConvertFrom','epochtime','TicksPerSecond',1e3,'TimeZone','Europe/Zurich','Format','dd-MMM-yyyy HH:mm:ss.SSS');

%% VISUALIZE DATA %%
% TODO: ridefinire i grafici in funzione dei nuovi dati
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

