%% PARAMS %%
% Log files directory
logsPath = './logs/';

% Name of csv file with extension
csvName = 'feeds.csv';

% Number of values to show
numPoints = 50;


%% GET DATA FROM CSV%%
path = strcat(logsPath,csvName);
opts = detectImportOptions(path);
T = readtable(path, opts);

% Get the last numPoints rows
T = T(max(1,(end-(numPoints - 1))):end, :);

% Convert created_at to datetime (inputformat like "2023-10-09T09:49:20+02:00")
timestampDate = datetime(T.created_at,'InputFormat','yyyy-MM-dd''T''HH:mm:ssXXX','TimeZone','Europe/Zurich');


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

