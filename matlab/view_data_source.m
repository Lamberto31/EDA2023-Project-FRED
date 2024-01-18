%% PARAMS %%
% Source (Use Constants to assign value)
    % Constants.SOURCE_LOCAL: get data from local csv
    % Constants.SOURCE_EXPORTED: get data from csv exported from ThingSpeak
    % Constants.SOURCE_REMOTE: get data from ThingSpeak
source = Constants.SOURCE_LOCAL;

% Number of values to show (from the end)
% If more than available it will show all the available values
numPoints = 100;

% Filter by state option
    % Constants.STATUS_ALL: show all the values
    % Constants.STATUS_EXPLORE: show only the values with status "Exploration"
    % Constants.STATUS_DATA_TRANSMISSION: show only the values with status "data transmission"
statusToView = Constants.STATUS_ALL;

% Filter by time ( consider data if time difference from previous < timeDiff)
    % Constants.TIME_ALL: don't filter by time difference
    % Constants.TIME_MINUTE: consider data if time difference from previous < 1 minute
    % Constants.TIME_HOUR: consider data if time difference from previous < 1 hour
    % Constants.TIME_DAY: consider data if time difference from previous > 1 day
maxTimeDiff = Constants.TIME_MINUTE;

% Name of csv file with extension
csvName = 'FRED_log_2023-12-30T18_55_28_debug_disabled.csv';
%csvName = 'feeds_new_bdt.csv';

%% BASIC CONFIGURATION FOR EACH SOURCE
switch source
    case Constants.SOURCE_LOCAL
        sourceInfo = "Local csv";
        logsPath = '../raspberry/logs/';
    case Constants.SOURCE_EXPORTED
        sourceInfo = "Csv exported from ThingSpeak";
        logsPath = './logs/';
    case Constants.SOURCE_REMOTE
        sourceInfo = "ThingSpeak";
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
    dataLength = height(T);
    % Time processing depends on local or exported choice
    if source == Constants.SOURCE_LOCAL
        % Extract timestamp and convert from epoch to datetime
        timestampEpoch = T.created_at;
        timestampDateGMT = datetime(timestampEpoch,'ConvertFrom','epochtime','TicksPerSecond',1e3,'TimeZone', 'Etc/GMT', 'Format','dd-MMM-yyyy HH:mm:ss.SSS');
        T.created_at = datetime(timestampDateGMT, 'TimeZone', 'Europe/Rome');
    else
        % Convert created_at to datetime (inputformat like "2023-10-09T09:49:20+02:00")
        T.created_at = datetime(T.created_at,'InputFormat','yyyy-MM-dd''T''HH:mm:ssXXX','TimeZone','Europe/Rome');
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

    % Build table (with empty status)
    % Get the length of data read from ThingSpeak
    dataLength = length(time1);
    status = strings(dataLength,1);
    T = table(time1, input, positionMeasure, velocityMeasure, positionEstimate, velocityEstimate, positionCovariance, velocityCovariance, customDistance, status);
    T.Properties.VariableNames = {'created_at', 'field1', 'field2', 'field3', 'field4', 'field5', 'field6', 'field7', 'field8', 'status'};
    T.created_at.TimeZone = 'Europe/Rome';

    % Status
    % Get status from Thingspeak with rest api (not available with thingSpeakRead)
    thingSpeakUrl = "https://api.thingspeak.com/channels/"+channelID+"/status.json?api_key="+readAPIKey+"&results="+dataLength+"&days=365";
    thingSpeakStatusAns = webread(thingSpeakUrl);
    feeds = thingSpeakStatusAns.feeds;
    % Insert feeds.status in T
    for i = 1:dataLength
        % Convert feeds_created_at in datetime
        feeds(i).created_at = datetime(feeds(i).created_at,'InputFormat','yyyy-MM-dd''T''HH:mm:ssXXX', 'TimeZone', 'Europe/Rome');
        % Find the matching T.created_at
        match = feeds(i).created_at == T.created_at;
        % Insert feeds(i).status in matching T.status
        T.status(match) = feeds(i).status;
    end
end


%% TIME FILTER
if maxTimeDiff ~= Constants.TIME_ALL
    lastIndex = 1;
    % Get the index of the last row that satisfies the time difference
    for i = dataLength:-1:2
        if seconds(T.created_at(i) - T.created_at(i-1)) > maxTimeDiff
            lastIndex = i;
            break;
        end
    end
    T = T(lastIndex:end,:);
end


%% STATUS FILTER
if statusToView ~= Constants.STATUS_ALL
    % Get the rows where status is statusToView
    T = T(T.status == statusToView,:);
end


%% VISUALIZE DATA %%
% Update dataLength
dataLength = height(T);

% Print some information
disp("Source: " + sourceInfo);
disp("Number of points: " + string(dataLength));
disp("Time difference between first and last timestamp: " + seconds(T.created_at(end) - T.created_at(1)) + " seconds");
disp("First timestamp: " + string(T.created_at(1)));
disp("Last timestamp: " + string(T.created_at(end)));

% Input
figure();
hold on;
plot(T.created_at, T.field1, '-', 'Color', [0.9290 0.6940 0.1250]);
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
hplot1 = plot(T.created_at, T.field2, '-o', 'DisplayName', 'Ultrasonic distance', 'Color', [0.4940 0.1840 0.5560]);
hplot2 = plot(T.created_at, T.field4, '-*', 'DisplayName', 'Position estimate', 'Color', [0.4660 0.6740 0.1880]);
title('Position');
legend([hplot1, hplot2]);
grid(ax1,'on')
hold off

% Velocity
ax2 = nexttile;
hold on
hplot3 = plot(T.created_at, T.field3, '-o', 'DisplayName', 'Optical pulses', 'Color', [0.4940 0.1840 0.5560]);
hplot4 = plot(T.created_at, T.field5, '-*', 'DisplayName', 'Velocity estimate', 'Color', [0.4660 0.6740 0.1880]);
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
hplot5 = plot(T.created_at, T.field6, '-', 'DisplayName', 'Position covariance');
title('Position Covariance');
grid(ax3,'on')
hold off

% Velocity
ax4 = nexttile;
hold on
hplot6 = plot(T.created_at, T.field7, '-', 'DisplayName', 'Velocity covariance');
title('Velocity Covariance');
grid(ax4,'on')
hold off

