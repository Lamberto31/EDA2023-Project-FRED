%% PARAMS %%
% Log files directory
logsPath = '../raspberry/logs/';

% Name of csv file with extension
csvName = 'FRED_log_2023-10-06T17_39_04.csv';


%% GET DATA FROM CSV%%
path = strcat(logsPath,csvName);
opts = detectImportOptions(path);
M = readmatrix(path, opts);

% Extract timestamp and convert from epoch to datetime
timestampEpoch = M(:,1);
timestampDate = datetime(timestampEpoch,'ConvertFrom','posixtime','TimeZone','Europe/Zurich','Format','dd-MMM-yyyy HH:mm:ss');


%% VISUALIZE DATA %%
% Layout definition
tiledlayout(3,1);

% Position
ax1 = nexttile;
hold on
hplot1 = plot(timestampDate, M(:,2), '-o', 'DisplayName', 'US');
hplot2 = plot(timestampDate, M(:,3), '-*', 'DisplayName', 'Optical');
hplot3 = plot(timestampDate, M(:,4), '-pentagram', 'DisplayName', 'Filtered');
title('Position');
legend([hplot1, hplot2, hplot3]);
grid(ax1,'on')
hold off

% Velocity
ax2 = nexttile;
hold on
hplot5 = plot(timestampDate, M(:,6), '-o', 'DisplayName', 'US');
hplot6 = plot(timestampDate, M(:,7), '-*', 'DisplayName', 'Optical');
hplot7 = plot(timestampDate, M(:,8), '-pentagram', 'DisplayName', 'Filtered');
title('Velocity');
legend([hplot1, hplot2, hplot3]);
grid(ax2,'on')
hold off

% Rps
ax3 = nexttile;
hplot4 = plot(timestampDate, M(:,5), '-*', 'DisplayName', 'Optical');
title('RPS');
legend(hplot4);
grid(ax3,'on')

