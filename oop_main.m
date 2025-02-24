clc; clearvars; close all; format compact

%{

TODO:

Add dependencies between methods (mainly statistics) to avoid user not
using them in order

Make it so that the object checks if starting position exists to do the
picker, not the loop
%}

%% Adding paths
addpath("src");
addpath("TrackerFiles");
addpath("Figures");
addpath("Images");


%% Determing File Names
FileNames = string(ls("TrackerFiles"));
FullFileNames = FileNames(3:end, :);
AbrevFileNames = erase(FullFileNames, ["_", ".txt"]);

%% Reading object

if exist("T_Results.mat", 'file') == 2
    load("T_Results.mat");
else

    %% Creating objects and loading data
    for i = numel(FullFileNames):-1:1

        t(i) = tracker_file(string(FullFileNames(i)));
        t(i).FileName = FullFileNames(i);
        t(i).TitleName = AbrevFileNames(i);

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

end
%% Adjusting the positions

for i = 1:numel(FullFileNames)
    t(i).PlotStrides
    t(i).AdjustStrides
    saveBool = 1;
    if saveBool
        save("T_Results.mat", 't');
    end

end
%% Saving the object

for i = 1:numel(FullFileNames)
    t(i).save2jpg;
    close all
end
