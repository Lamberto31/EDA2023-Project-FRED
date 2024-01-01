classdef Constants
    properties (Constant = true)
        % Source of the data
        SOURCE_LOCAL = 0;
        SOURCE_EXPORTED = 1;
        SOURCE_REMOTE = 2;
        % Status of the data
        STATUS_ALL = "All"
        STATUS_EXPLORE = "Exploration";
        STATUS_DATA_TRANSMISSION = "Data transmission"
        % Time constants
        TIME_ALL = 0;
        TIME_MINUTE = 60;
        TIME_HOUR = 3600;
        TIME_DAY = 86400;
    end
end