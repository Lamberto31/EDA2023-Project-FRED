%% PARAMS %%
% Log files directory
logsPath = './logs/';

% Name of csv file with extension
csvName = 'feeds_new_bdt.csv';

% Number of values to show
numPoints = 50;

% Exploration status name
explorationStatusName = "Exploration";


%% GET DATA FROM CSV%%
path = strcat(logsPath,csvName);
opts = detectImportOptions(path);
T = readtable(path, opts);

% Get the last numPoints rows
T = T(max(1,(end-(numPoints - 1))):end, :);

% Get the rows where status is explorationStatusName
T = T(T.status == explorationStatusName,:);

% Convert created_at to datetime (inputformat like "2023-10-09T09:49:20+02:00")
timestampDate = datetime(T.created_at,'InputFormat','yyyy-MM-dd''T''HH:mm:ssXXX','TimeZone','Europe/Rome');


%% VISUALIZE DATA %%

% Input
figure();
hold on;
plot(timestampDate, T.field1, '-', 'Color', [0.9290 0.6940 0.1250]);
title('Input');
xlabel('Time');
ylabel('Input');
grid on

% State (measured and estimated)
figure();
% Layout definition
tiledlayout(2,1);

% Position
ax1 = nexttile;
hold on
hplot1 = plot(timestampDate, T.field2, '-o', 'DisplayName', 'Ultrasonic distance', 'Color', [0.4940 0.1840 0.5560]);
hplot2 = plot(timestampDate, T.field4, '-*', 'DisplayName', 'Position estimate', 'Color', [0.4660 0.6740 0.1880]);
title('Position');
legend([hplot1, hplot2]);
grid(ax1,'on')
hold off

% Velocity
ax2 = nexttile;
hold on
hplot3 = plot(timestampDate, T.field3, '-o', 'DisplayName', 'Optical pulses', 'Color', [0.4940 0.1840 0.5560]);
hplot4 = plot(timestampDate, T.field5, '-*', 'DisplayName', 'Velocity estimate', 'Color', [0.4660 0.6740 0.1880]);
title('Velocity');
legend([hplot3, hplot4]);
grid(ax2,'on')
hold off

% State Covariance
figure();
% Layout definition
tiledlayout(2,1);

% Position
ax3 = nexttile;
hold on
hplot5 = plot(timestampDate, T.field6, '-', 'DisplayName', 'Position covariance');
title('Position Covariance');
grid(ax3,'on')
hold off

% Velocity
ax4 = nexttile;
hold on
hplot6 = plot(timestampDate, T.field7, '-', 'DisplayName', 'Velocity covariance');
title('Velocity Covariance');
grid(ax4,'on')
hold off

