%% PARAMS %%
% Credentials file
credentialFile = 'credentials.csv';

% Number of values to show
numPoints = 50;


%% EXTRACT CREDENTIALS
opts = detectImportOptions("credentials.csv");
T = readtable("credentials.csv", opts);
channelID = T.channelid;
readAPIKey = string(T.readapiky);


%% FIELDS %%
% Input
inputId = 1;

% Position
positionMeasureId = 2;
positionEstimateId = 4;
positionCovarianceId = 6;

% Velocity
velocityMeasureId = 3;
velocityEstimateId = 5;
velocityCovarianceId = 7;


%% GET DATA FROM THINGSPEAK CHANNEL%%
% Input
[input, time1] = thingSpeakRead(channelID, 'Field', inputId, 'NumPoints', numPoints, 'ReadKey', readAPIKey);

% Position
[positionMeasure, time2] = thingSpeakRead(channelID, 'Field', positionMeasureId, 'NumPoints', numPoints, 'ReadKey', readAPIKey);
[positionEstimate, time4] = thingSpeakRead(channelID, 'Field', positionEstimateId, 'NumPoints', numPoints, 'ReadKey', readAPIKey);
[positionCovariance, time6] = thingSpeakRead(channelID, 'Field', positionCovarianceId, 'NumPoints', numPoints, 'ReadKey', readAPIKey);

% Velocity
[velocityMeasure, time3] = thingSpeakRead(channelID, 'Field', velocityMeasureId, 'NumPoints', numPoints, 'ReadKey', readAPIKey);
[velocityEstimate, time5] = thingSpeakRead(channelID, 'Field', velocityEstimateId, 'NumPoints', numPoints, 'ReadKey', readAPIKey);
[velocityCovariance, time7] = thingSpeakRead(channelID, 'Field', velocityCovarianceId, 'NumPoints', numPoints, 'ReadKey', readAPIKey);


%% VISUALIZE DATA %%

% Input
figure();
hold on;
plot(time1, input, '-', 'Color', [0.9290 0.6940 0.1250]);
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
hplot1 = plot(time2, positionMeasure, '-o', 'DisplayName', 'Ultrasonic distance', 'Color', [0.4940 0.1840 0.5560]);
hplot2 = plot(time4, positionEstimate, '-*', 'DisplayName', 'Position estimate', 'Color', [0.4660 0.6740 0.1880]);
title('Position');
legend([hplot1, hplot2]);
grid(ax1,'on')
hold off

% Velocity
ax2 = nexttile;
hold on
hplot3 = plot(time3, velocityMeasure, '-o', 'DisplayName', 'Optical pulses', 'Color', [0.4940 0.1840 0.5560]);
hplot4 = plot(time5, velocityEstimate, '-*', 'DisplayName', 'Velocity estimate', 'Color', [0.4660 0.6740 0.1880]);
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
hplot5 = plot(time6, positionCovariance, '-', 'DisplayName', 'Position covariance');
title('Position Covariance');
grid(ax3,'on')
hold off

% Velocity
ax4 = nexttile;
hold on
hplot6 = plot(time7, velocityCovariance, '-', 'DisplayName', 'Velocity covariance');
title('Velocity Covariance');
grid(ax4,'on')
hold off
