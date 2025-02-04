clc; clearvars -except Data; close all; format compact;

addpath("SlippingFunctions");
addpath("TrackerFiles");

%{

TO-DO

Need to re-write this so that multiple trials of the same frequency can be
used.  Can concatenate in the "Sorting each trial by stride based on time
alone" section.  Also in this section, ensure correct number of plots are
being made (comment is in there with more detail)

May also need to normalize all of the x values.

Eventually, move sections to their own functions
%}
%% Sorting file Names
FileNames = string(ls("TrackerFiles"));
FullFileNames = FileNames(3:end, :);
AbrevFileNames = erase(FullFileNames, ["_", ".txt"]);

%% Putting The Tracker Data into fields
% % Note: this function will reset the start index as of now
% for i = 1:numel(FullFileNames)
%
%     Data.(AbrevFileNames(i)) = parse_tracker_data(string(FullFileNames(i)));
%
% end

%% Finding out where the starting index is for each plot

for i = 1:numel(FullFileNames)
    % Only 'ask' to pick a starting point if there isn't one saved yet
    if isempty(Data.(AbrevFileNames(i)).StartIndex)
        % call frame parser
        StartIndex = frame_picker(Data.(AbrevFileNames(i)));
        Data.(AbrevFileNames(i)).StartIndex = StartIndex;
        close
    end
end

%% Sorting each trial by stride based on time alone

for i = 1:numel(FullFileNames)
    % Need to figure out what the frequency of the take is. Relies on files
    % being named consistently
    if contains(FullFileNames(i), "_1p0Hz")
        frequency = 1.0;
    elseif contains(FullFileNames(i), "_2p5Hz")
        frequency = 2.5;
    elseif contains(FullFileNames(i), "_4p0Hz")
        frequency = 4.0;
    end
    Data.(AbrevFileNames(i)).frequency = frequency;

    % Now, splitting each stride into its own field
    StartIndex = Data.(AbrevFileNames(i)).StartIndex;
    t_not = Data.(AbrevFileNames(i)).t(StartIndex);
    t_end = Data.(AbrevFileNames(i)).t(end);
    dt = 1/frequency;
    t_vals = t_not:dt:t_end;

    % [~, t_vals_idx_1] = find(t_vals <= 1.01*Data.(AbrevFileNames(i)).t );

    %\cite{https://www.mathworks.com/matlabcentral/answers/152301-find-closest-value-in-array#comment_2806253}
    [~, t_vals_idx] = min(abs(Data.(AbrevFileNames(i)).t - t_vals));
    Data.(AbrevFileNames(i)).t_vals = t_vals;


    figure();
    hold on
    num_trials = numel(t_vals);

    % Make sure not ending too soon, add end time?
    for k = 2:num_trials

        curX = Data.(AbrevFileNames(i)).x(t_vals_idx(k-1):t_vals_idx(k));
        curT = Data.(AbrevFileNames(i)).t(t_vals_idx(k-1):t_vals_idx(k));

        % Normalizing each stride
        curX = curX - curX(1);
        curT = curT - curT(1);

        Data.(AbrevFileNames(i)).(strcat("Stride_", num2str(k-1))).x = curX;
        Data.(AbrevFileNames(i)).(strcat("Stride_", num2str(k-1))).t = curT;

        subplot(round(num_trials/2), 2, k-1);
        plot(curT, curX);

    end

end

%% Taking statistical data from each stride
% Could combine this with the previous step to help with speed

close all

for i = 1:numel(FullFileNames)
    cur_fields = fieldnames(Data.(AbrevFileNames(i)));
    cur_num_strides = sum(contains(cur_fields, "Stride"));

    % x_tot is a matrix which contains the x-values for each stride for a
    % given frequency. Each row represents a stride, and each column
    % represents x(t) for that stride
    x_tot = [];
    for k = 1:cur_num_strides
        cur_x = Data.(AbrevFileNames(i)).(strcat("Stride_", num2str(k))).x;
        % This works because we're assuming dt is constant
        x_tot = [x_tot; cur_x'];

    end

    % Taking the mean and std dev by column (each dt)
    mean_x = mean(x_tot, 1);
    std_x = std(x_tot, 0, 1);

    % Finding maxes
    [~, max_per_column] = max(x_tot, [], 1);
    % I'm sure there's a faster way to do this. The idea is that the
    % max_per_column value above stores all of the indices (row numbers)
    % that the max occurs in each column.  Then, I'm going through x_tot to
    % pick out what this value is.
    for z = 1:numel(max_per_column)
        max_x_vals(z) = x_tot(max_per_column(z), z);
    end
    % Finding mins
    [~, min_per_column] = min(x_tot, [], 1);
    for z = 1:numel(min_per_column)
        min_x_vals(z) = x_tot(min_per_column(z), z);
    end


    % Storing to the struct
    Data.(AbrevFileNames(i)).mean_x = mean_x;
    Data.(AbrevFileNames(i)).std_x = std_x;

    % Always the same rang of time bc assuming dt's constant
    cur_x_axis = Data.(AbrevFileNames(i)).(strcat("Stride_", num2str(k))).t';

    figure();
    hold on
    fill([cur_x_axis, flip(cur_x_axis)], [mean_x + std_x, flip(mean_x - std_x)], [0.8 0.8 0.8]); % \cite{https://www.mathworks.com/matlabcentral/answers/1928100-create-plot-with-shaded-standard-deviation-but#answer_1192320}
    plot(cur_x_axis, mean_x, '--k');
    plot(cur_x_axis, max_x_vals, '--b');
    plot(cur_x_axis, min_x_vals, '--r');
    legend('', "Mean", "Max", "Min");
    xlabel("Time(s)");
    ylabel("Change in X-Position(m)");
    title(strcat("Normalized X-Position for ", num2str(frequency), "Hz"));
    hold off

    clear max_x_vals min_x_vals

end



