%{

Use this file to plot the parse tracker data and plot the results

jjv20@fsu.edu
1/23/25

%}
clc; clear all; close all; format compact



%% Sorting file Names
FileNames = string(ls("TrackerFiles"));
FullFileNames = FileNames(3:end, :);
AbrevFileNames = erase(FullFileNames, ["_", ".txt"]);

%% Main loop to plot each data set
for i = 1:numel(FullFileNames)
    data = parse_tracker_data(strcat("TrackerFiles/",FullFileNames(i)));
    neg_dx_indices = find_slip(data);
    plot_data(data, i, AbrevFileNames, neg_dx_indices);
end

%% Functions

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% Function for plotting data
function plot_data(data, i, AbrevFileNames, neg_dx_indices)

    figure(i)
    hold on
    plot(data.x, data.t);
    xlabel("X-Position (m)")
    ylabel("Time (s)")
    title(strcat("X Position for ", AbrevFileNames(i)))
    scatter(data.x(neg_dx_indices), data.t(neg_dx_indices), '*r');
    legend("Foot Position", "Slippage Detected", 'location', 'northwest');
    grid on;
    hold off
    
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
This is a quick and dirty solution to this.
    %}

    if cur_col_size == 7
        data.t = temp(:, 1);
        data.x = temp(:, 2);
        data.y = temp(:, 3);        
        data.vx = temp(:, 4);
        data.vy = temp(:, 5);
        data.ax = temp(:, 6);
        data.ay = temp(:, 7);

    elseif cur_col_size == 10
        data.t = temp(:, 1);
        data.x = temp(:, 2);
        data.y = temp(:, 3);
        data.r = temp(:, 4);
        data.vx = temp(:, 5);
        data.vy = temp(:, 6);
        data.v = temp(:, 7);
        data.ax = temp(:, 8);
        data.ay = temp(:, 9);
        data.a = temp(:, 10);
    end
end