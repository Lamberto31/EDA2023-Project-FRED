%% PARAMS %%
% Source (Use Constants to assign value)
source = Constants.SOURCE_REMOTE;

% Number of values to show (from the end)
numPoints = 10;

% Filter by state option
statusToView = Constants.STATUS_EXPLORE;

% Take only the last execution option
% TODO

% Name of csv file with extension
csvName = 'FRED_log_2023-12-30T18_55_28_debug_disabled.csv';
%csvName = 'feeds_new_bdt.csv';

%% BASIC CONFIGURATION FOR EACH SOURCE
switch source
    case Constants.SOURCE_LOCAL
        disp("Getting data from local csv")
        logsPath = '../raspberry/logs/';
    case Constants.SOURCE_EXPORTED
        disp("Getting data from csv exported from ThingSpeak")
        logsPath = './logs/';
    case Constants.SOURCE_REMOTE
        disp('Getting data from ThingSpeak')
        cred_opts = detectImportOptions("credentials.csv");
        C = readtable("credentials.csv", cred_opts);
        channelID = C.channelid;
        readAPIKey = string(C.readapiky);
    otherwise
        disp("Invalid source");
end


%% GET DATA FROM SOURCE
% If is local get data from local file
if source == Constants.SOURCE_LOCAL || source == Constants.SOURCE_EXPORTED
    % Get data from local file
    path = strcat(logsPath,csvName);
    opts = detectImportOptions(path);
    T = readtable(path, opts);

    % Get the last numPoints rows
    T = T(max(1,(end-(numPoints - 1))):end, :);
    % Time processing depends on local or exported choice
    if source == Constants.SOURCE_LOCAL
        % Extract timestamp and convert from epoch to datetime
        timestampEpoch = T.created_at;
        timestampDateGMT = datetime(timestampEpoch,'ConvertFrom','epochtime','TicksPerSecond',1e3,'TimeZone', 'Etc/GMT', 'Format','dd-MMM-yyyy HH:mm:ss.SSS');
        timestampDate = datetime(timestampDateGMT, 'TimeZone', 'Europe/Rome');
    else
        % Convert created_at to datetime (inputformat like "2023-10-09T09:49:20+02:00")
        timestampDate = datetime(T.created_at,'InputFormat','yyyy-MM-dd''T''HH:mm:ssXXX','TimeZone','Europe/Rome');
    end
% Get data from ThingSpeak
else
    % Fields definition
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
    % Custom distance
    customDistanceId = 8;
    
    % Get data from ThingSpeak
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

    % Custom distance
    [customDistance, time8] = thingSpeakRead(channelID, 'Field', customDistanceId, 'NumPoints', numPoints, 'ReadKey', readAPIKey);

    % Status
    % TODO: Capire come ottenere status da thingSpeakRead

    % Build table
    T = table(time1, input, positionMeasure, velocityMeasure, positionEstimate, velocityEstimate, positionCovariance, velocityCovariance, customDistance, status);
    T.Properties.VariableNames = {'created_at', 'field1', 'field2', 'field3', 'field4', 'field5', 'field6', 'field7', 'field8', 'status'};
    timestampDate = T.created_at;
end


%% STATUS FILTER
if statusToView ~= Constants.STATUS_ALL
    % Get the rows where status is statusToView
    T = T(T.status == statusToView,:);
end


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

