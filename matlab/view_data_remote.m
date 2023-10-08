%% PARAMS %%
% Credentials file
credentialFile = 'credentials.csv';

% Number of values to show
numPoints = 37;


%% EXTRACT CREDENTIALS
opts = detectImportOptions("credentials.csv");
T = readtable("credentials.csv", opts);
channelID = T.channelid;
readAPIKey = string(T.readapiky);


%% FIELDS %%
% Position
positionUSId = 1;
positionOpticalId = 2;
positionFilteredId = 3;

% Velocity
velocityUSId = 5;
velocityOpticalId = 6;
velocityFilteredId = 7;

% RPS
rpsId = 4;


%% GET DATA FROM THINGSPEAK CHANNEL%%
% Position
[positionUS, time1] = thingSpeakRead(channelID, 'Field', positionUSId, 'NumPoints', numPoints, 'ReadKey', readAPIKey);
[positionOptical, time2] = thingSpeakRead(channelID, 'Field', positionOpticalId, 'NumPoints', numPoints, 'ReadKey', readAPIKey);
[positionFiltered, time3] = thingSpeakRead(channelID, 'Field', positionFilteredId, 'NumPoints', numPoints, 'ReadKey', readAPIKey);

% Velocity
[velocityUS, time5] = thingSpeakRead(channelID, 'Field', velocityUSId, 'NumPoints', numPoints, 'ReadKey', readAPIKey);
[velocityOptical, time6] = thingSpeakRead(channelID, 'Field', velocityOpticalId, 'NumPoints', numPoints, 'ReadKey', readAPIKey);
[velocityFiltered, time7] = thingSpeakRead(channelID, 'Field', velocityFilteredId, 'NumPoints', numPoints, 'ReadKey', readAPIKey);

% RPS
[rps, time4] = thingSpeakRead(channelID, 'Field', rpsId, 'NumPoints', numPoints, 'ReadKey', readAPIKey);


%% VISUALIZE DATA %%
% Layout definition
tiledlayout(3,1);

% Position
ax1 = nexttile;
hold on
hplot1 = plot(time1, positionUS, '-o', 'DisplayName', 'US');
hplot2 = plot(time2, positionOptical, '-*', 'DisplayName', 'Optical');
hplot3 = plot(time3, positionFiltered, '-pentagram', 'DisplayName', 'Filtered');
title('Position');
legend([hplot1, hplot2, hplot3]);
grid(ax1,'on')
hold off

% Velocity
ax2 = nexttile;
hold on
hplot5 = plot(time5, velocityUS, '-o', 'DisplayName', 'US');
hplot6 = plot(time6, velocityOptical, '-*', 'DisplayName', 'Optical');
hplot7 = plot(time7, velocityFiltered, '-pentagram', 'DisplayName', 'Filtered');
title('Velocity');
legend([hplot5, hplot6, hplot7]);
grid(ax2,'on')
hold off

% Rps
ax3 = nexttile;
hplot4 = plot(time4, rps, '-*', 'DisplayName', 'Optical');
title('RPS');
legend(hplot4);
grid(ax3,'on')

