clc; clear; close all; format compact

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

