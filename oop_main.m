clc; clearvars -except t starting_points; close all; format compact

%% Adding paths
addpath("SlippingFunctions");
addpath("TrackerFiles");

%% Determing File Names
FileNames = string(ls("TrackerFiles"));
FullFileNames = FileNames(3:end, :);
AbrevFileNames = erase(FullFileNames, ["_", ".txt"]);

%% Creating objects and loading data
for i = numel(FullFileNames):-1:1

    t(i) = tracker_file(string(FullFileNames(i)));
    t(i).FileName = FullFileNames(i);

end

%% Asking for Starting Point, Creating Strides

for i = numel(FullFileNames):-1:1

    cur_t = t(i);

    cur_t.PickStartingPoint
    cur_t.CreateStridePredicitons
    % cur_t.PlotStrides
    % cur_t.AdjustStrides
    clear cur_t
end
