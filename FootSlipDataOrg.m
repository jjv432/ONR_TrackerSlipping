clc; clearvars -except Data; close all; format compact;

addpath("SlippingFunctions");
addpath("TrackerFiles");

%% Sorting file Names
FileNames = string(ls("TrackerFiles"));
FullFileNames = FileNames(3:end, :);
AbrevFileNames = erase(FullFileNames, ["_", ".txt"]);

%% Putting The Tracker Data into fields

for i = 1:numel(FullFileNames)

    Data.(AbrevFileNames(i)) = parse_tracker_data(string(FullFileNames(i)));

end




