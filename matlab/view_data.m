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

% Input
figure();
hold on;
plot(timestampDate, T.field1, '-square');
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
hplot1 = plot(timestampDate, T.field2, '-o', 'DisplayName', 'Ultrasonic distance');
hplot2 = plot(timestampDate, T.field4, '-*', 'DisplayName', 'Position estimate');
title('Position');
legend([hplot1, hplot2]);
grid(ax1,'on')
hold off

% Velocity
ax2 = nexttile;
hold on
hplot3 = plot(timestampDate, T.field3, '-o', 'DisplayName', 'Optical pulses');
hplot4 = plot(timestampDate, T.field5, '-*', 'DisplayName', 'Velocity estimate');
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

