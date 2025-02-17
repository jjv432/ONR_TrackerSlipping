clc; clearvars -except t; close all; format compact

%% Adding paths
addpath("src");
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

%% Statistics

for i = 1:numel(FullFileNames)
    cur_t = t(i);

    cur_t.GenerateStatistics
    cur_t.PlotStatistics


end