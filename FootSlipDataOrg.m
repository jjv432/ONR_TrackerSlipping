clc; clearvars -except Data; close all; format compact;

addpath("SlippingFunctions");
addpath("TrackerFiles");

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

    % Now, splitting each stride into its own field
    StartIndex = Data.(AbrevFileNames(i)).StartIndex;
    t_not = Data.(AbrevFileNames(i)).t(StartIndex);
    t_end = Data.(AbrevFileNames(i)).t(end);
    dt = 1/frequency;
    t_vals = t_not:dt:t_end;

    % [~, t_vals_idx_1] = find(t_vals <= 1.01*Data.(AbrevFileNames(i)).t );
  
    %\cite{https://www.mathworks.com/matlabcentral/answers/152301-find-closest-value-in-array#comment_2806253}
    [~, t_vals_idx] = min(abs(Data.(AbrevFileNames(i)).t - t_vals));
  

    figure();
    hold on
    num_trials = numel(t_vals);
    
    for k = 2:num_trials
        
        curX = Data.(AbrevFileNames(i)).x(t_vals_idx(k-1):t_vals_idx(k));
        curT = Data.(AbrevFileNames(i)).t(t_vals_idx(k-1):t_vals_idx(k));
        
        Data.(AbrevFileNames(i)).(strcat("Stride_", num2str(k-1))).x = curX;
        Data.(AbrevFileNames(i)).(strcat("Stride_", num2str(k-1))).t = curT;

        subplot(round(num_trials/2), 2, k-1);
        plot(curT, curX);
        
    end

end



