%{

Use this file to plot the parse tracker data and plot the results

jjv20@fsu.edu
1/23/25

%}
clc; clearvars -except data; close all; format compact

% Some hard-coded values to make zoomed-in plots
beginningIndex = [32, 36, 70];
endIndex = [92, 62, 85];


%% Sorting file Names
FileNames = string(ls("TrackerFiles"));
FullFileNames = FileNames(3:end, :);
AbrevFileNames = erase(FullFileNames, ["_", ".txt"]);

%% Main loop to plot each data set
for i = 1:numel(FullFileNames)
    data = parse_tracker_data(strcat("TrackerFiles/",FullFileNames(i)));
    neg_dx_indices = find_slip(data);
    chop_index = chop_data(data, 3.5);
    % plot_data(data, i, AbrevFileNames, neg_dx_indices, chop_index);
    plot_gait(data, i, AbrevFileNames, neg_dx_indices, beginningIndex(i), endIndex(i))
end

%% Functions

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% Function for plotting one gait cycle

function plot_gait(data, i, AbrevFileNames, neg_dx_indices, beginningIndex, endIndex)
    figure()
    hold on
    xlim([data.t(beginningIndex), data.t(endIndex)]);  
    plot(data.t(beginningIndex:endIndex), data.x(beginningIndex:endIndex), "LineWidth",3.5);
    ylabel("X-Position (m)")
    xlabel("Time (s)")
    title(strcat("X Position for ", AbrevFileNames(i)))
    % scatter(data.t(neg_dx_indices), data.x(neg_dx_indices), 75, 'xr', 'LineWidth',1.5);
    legend("Foot Position", "Slippage Detected", 'location', 'northwest');
    grid on;
    hold off


end
%% Function for plotting data
function plot_data(data, i, AbrevFileNames, neg_dx_indices, chop_index)

    figure()
    hold on
    xlim([0, data.t(chop_index)]);
    plot(data.t(1:chop_index), data.x(1:chop_index), "LineWidth",3.5);
    ylabel("X-Position (m)")
    xlabel("Time (s)")
    title(strcat("X Position for ", AbrevFileNames(i)))
    scatter(data.t(neg_dx_indices), data.x(neg_dx_indices), 75, 'xr', 'LineWidth',1.5);
    legend("Foot Position", "Slippage Detected", 'location', 'northwest');
    grid on;
    hold off
    
end

%% Function for chopping off data at a given time

function chop_index = chop_data(unchopped_data, end_time)

    chop_index = find(unchopped_data.t >= end_time, 1, 'first');


end
%% Function for finding dx/dt

function neg_vx_indices = find_slip(data)

   vx = data.vx;
   neg_vx_indices = find(vx<0);

end

%% Function for sorting data

function data = parse_tracker_data(input_file)

    % Read file
    temp = readmatrix(input_file);

    cur_col_size = size(temp, 2);

    % Sort file into each column
    %{
For whatever reason, not all of the files had the same number of columns.
This is a quick and dirty solution to this.  The small ones don't have
columns for the following values: r, v, a.  
    %}

    if cur_col_size == 7
        data.t = temp(:, 1)- temp(1,1);
        data.x = temp(:, 2)- temp(1,2);
        data.y = temp(:, 3)- temp(1,3);        
        data.vx = temp(:, 4);
        data.vy = temp(:, 5);
        data.ax = temp(:, 6);
        data.ay = temp(:, 7);

    elseif cur_col_size == 10
        data.t = temp(:, 1)- temp(1,1);
        data.x = temp(:, 2)- temp(1,2);
        data.y = temp(:, 3)- temp(1,3);
        data.r = temp(:, 4);
        data.vx = temp(:, 5);
        data.vy = temp(:, 6);
        data.v = temp(:, 7);
        data.ax = temp(:, 8);
        data.ay = temp(:, 9);
        data.a = temp(:, 10);
    end
end