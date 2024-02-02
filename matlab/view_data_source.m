% View data FRED

%% RESET INIZIALE
clear;
close all;


%% PARAMS %%
% Source (Use Constants to assign value)
    % Constants.SOURCE_LOCAL: get data from local csv
    % Constants.SOURCE_EXPORTED: get data from csv exported from ThingSpeak
    % Constants.SOURCE_REMOTE: get data from ThingSpeak
source = Constants.SOURCE_LOCAL;

% Number of values to show (from the end)
% If more than available it will show all the available values
numPoints = 500;

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

% Get multiple experiments (different objective in one explore mode)
% This change only the textual output
    % true to show a tabular view with each experiment
    % false to show just textual results referred to first and last moment
    % maxObjectiveNumber to limit the table columns if multipleObj true
multipleObj = true;
maxObjectiveNumber = 5;

% Name of csv file with extension
csvName = 'FRED_log_2024-02-01T17_56_52_final_filter.csv';
%csvName = 'feeds_new_bdt.csv';

%% PHYSICAL CONSTANST AND COMPUTATIONS
% Used for graphs
% INPUT
M = 0.731;
Vp = 5.85;
v_max = 71.3456;
t_0 = 0.3924;

% Input veloce
C_fast = 255; 
% Input lento
C_slow = 100;

% Computed
b = 5*(M/t_0);
eta_V = v_max*(b/Vp);
kappa = eta_V*Vp/255;

v_fast = kappa*C_fast/b;
v_slow = kappa*C_slow/b;
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
else
    % Remove rows where status is "Connected" or "Disconnected"
    T = T(T.status ~= "Connected",:);
    T = T(T.status ~= "Disconnected",:);
end


%% GET DIFFERENT EXPERIMENTS OBJECTIVES
if multipleObj
    % Get last obj value and index for each objective
    last_index = diff(T.field8) ~= 0;
    last_index(end+1) = 1;
    last_index = find(last_index);
    obj = T.field8(last_index);
    % Get first index for each objective
    first_index = [1, ((last_index + 1))']';
    first_index = first_index(1:end-1);

    % Build table to show
    % Get only useful rows and sort
    T_disp = T([first_index last_index],:);
    T_disp = sortrows(T_disp, "created_at");
    % Get only useful column
    T_disp = T_disp(:,[5, 6, 9]);
    % Add column diff
    T_disp.diff = abs(T_disp.field8 - T_disp.field4);
    % Rearrange table rows
    for i = 1:2:(2*height(obj))
        last_p(i:i+1) = [T_disp.field4(i+1)];
        last_v(i:i+1) = [T_disp.field5(i+1)];
        diff_obj(i:i+1) = [T_disp.diff(i+1)];
    end
    T_disp.lastp = (last_p)';
    T_disp.lastv = (last_v)';
    T_disp.diff = (diff_obj)';
    % Drop useless rows
    T_disp = T_disp(1:2:end,:);
    % Rearrange table columns
    T_disp = movevars(T_disp, "field8", "Before", "field4");
    T_disp = movevars(T_disp, "diff", "After", "lastv");
    T_disp = renamevars(T_disp, ["field4", "field5", "field8", "diff", "lastp", "lastv"], ...
                                ["Estimated starting position", "Estimated starting velocity", "Objective position", "Estimated position error (wrt obj)", "Estimated final position", "Estimated final velocity"]);
    
    % Transpose for readability
    T_array = table2array(T_disp);
    T_disp_transpose = array2table(T_array.');
    T_disp_transpose.Properties.RowNames = T_disp.Properties.VariableNames;
    T_disp_transpose.Properties.VariableNames = string(obj);
    % Redefine table for decimals

    % Set decimal precision
    n_decimal = 4;
    % Create a new table
    T_disp_decimal = varfun(@(x) num2str(x, ['%' sprintf('.%df', n_decimal)]), T_disp_transpose);
    % Preserve the variable names and the row names in the original table
    T_disp_decimal.Properties.VariableNames = T_disp_transpose.Properties.VariableNames;
    T_disp_decimal.Properties.RowNames = T_disp_transpose.Properties.RowNames;

    % Truncate if obj > maxObjectiveNumber
    if height(obj) > maxObjectiveNumber
        T_disp_decimal = T_disp_decimal(:,end-(maxObjectiveNumber-1):end);
    end
end


%% RESULTS %%
% Update dataLength
dataLength = height(T);

% Print some information
disp("Source: " + sourceInfo);
disp("Number of points: " + string(dataLength));
disp("Time difference between first and last timestamp: " + seconds(T.created_at(end) - T.created_at(1)) + " seconds");
disp("First timestamp: " + string(T.created_at(1)));
disp("Last timestamp: " + string(T.created_at(end)));

% Print results
disp(" ");
if multipleObj
    disp("Results:")
    disp(T_disp_decimal);
else
    obj = T.field8(end); %#ok<UNRCH>
    disp("Objective position: " + string(obj));
    disp("Estimated starting position: " + string(T.field4(1)));
    disp("Estimated starting velocity: " + string(T.field5(1)));
    disp("Estimated final position: " + string(T.field4(end)));
    disp("Estimated position error (wrt obj): " + string(abs(obj - T.field4(end))));
    disp("Estimated final velocity: " + string(T.field5(end)));
end


%% GRAPHS
% Input
figure;
plot(T.created_at, T.field1, '-', 'Color', 'm'); hold on;
title('Input');
xlabel('Time');
ylabel('Input');

% Position
figure;
hplot1 = plot(T.created_at, T.field2, 'DisplayName', 'Ultrasonic distance', 'Color', 'r'); hold on
hplot2 = plot(T.created_at, T.field4, 'DisplayName', 'Position estimate', 'Color', 'b');
hplot3 = plot(T.created_at, T.field8, 'DisplayName', 'Objective');
title('Position');
xlabel('Time');
ylabel('Position [cm]');
legend([hplot1, hplot2, hplot3]);
grid on

% Velocity
figure;
hplot4 = plot(T.created_at, T.field3, 'DisplayName', 'Optical pulses', 'Color', 'r'); hold on;
hplot5 = plot(T.created_at, T.field5, 'DisplayName', 'Velocity estimate', 'Color', 'b');
hplot6 = plot(T.created_at, ones(1,dataLength)*-v_fast, 'DisplayName', '(-)Max fast speed');
hplot7 = plot(T.created_at, ones(1,dataLength)*-v_slow, 'DisplayName', '(-)Max slow speed');
hplot8 = plot(T.created_at, ones(1,dataLength)*v_fast, 'DisplayName', 'Max fast speed');
hplot9 = plot(T.created_at, ones(1,dataLength)*v_slow, 'DisplayName', 'Max slow speed');
title('Velocity');
xlabel('Time');
ylabel('Velocity [cm/s]');
legend([hplot4, hplot5, hplot6, hplot7, hplot8, hplot9]);
grid on

% Trajectory
figure();
plot(0, 0, 'o'); hold on;
plot(T.field4(1), 0, '+');
plot(T.field4(1:end), zeros(dataLength,1), 'g');
plot(T.field4(end), 0, 'x');
plot(obj, 0, '*');
axis equal;
grid on;
legend('Obstacle', 'Start poisition', 'Trajectory', 'Final position', 'Objective');
xlabel('Position [cm]')
title('Trajectory');

% State Covariance
figure();
% Layout definition
tiledlayout(2,1);

% Position
ax3 = nexttile;
hplot10 = plot(T.created_at, T.field6, '-', 'DisplayName', 'Position covariance'); hold on;
title('Position Covariance');
xlabel('Time');
ylabel('[cm^2]')
grid(ax3,'on')
hold off

% Velocity
ax4 = nexttile;
hplot11 = plot(T.created_at, T.field7, '-', 'DisplayName', 'Velocity covariance'); hold on;
title('Velocity Covariance');
xlabel('Time');
ylabel('[(cm/s)^2]');
grid(ax4,'on')
hold off

