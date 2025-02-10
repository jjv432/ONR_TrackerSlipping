function [Data, FullFileNames, AbrevFileNames] = makeNewDataStruct()

    % Sorting file Names
    FileNames = string(ls("TrackerFiles"));
    FullFileNames = FileNames(3:end, :);
    AbrevFileNames = erase(FullFileNames, ["_", ".txt"]);

    %% Putting The Tracker Data into fields

    if ~exist('Data', 'var')
        % Note: this function will reset the start index as of now
        for i = 1:numel(FullFileNames)

            Data.(AbrevFileNames(i)) = parse_tracker_data(string(FullFileNames(i)));

        end
    end

    %% Finding out where the starting index is for each plot

    for i = 1:numel(FullFileNames)
        % Only 'ask' to pick a starting point if there isn't one saved yet
        if isempty(Data.(AbrevFileNames(i)).StartIndex)
            % call frame parser
            StartIndex = frame_picker(Data.(AbrevFileNames(i)), (AbrevFileNames(i)));
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
        elseif contains(FullFileNames(i), "_0p5Hz")
            frequency = 0.5788; % using fft for orignal file
        end
        Data.(AbrevFileNames(i)).frequency = frequency;

        % Now, splitting each stride into its own field
        StartIndex = Data.(AbrevFileNames(i)).StartIndex;
        t_not = Data.(AbrevFileNames(i)).t(StartIndex);
        t_end = Data.(AbrevFileNames(i)).t(end);
        dt = 1/frequency;
        t_vals = t_not:dt:t_end;


        %\cite{https://www.mathworks.com/matlabcentral/answers/152301-find-closest-value-in-array#comment_2806253}
        [~, t_vals_idx] = min(abs(Data.(AbrevFileNames(i)).t - t_vals));
        Data.(AbrevFileNames(i)).t_vals = t_vals;


        % figure();
        % hold on
        num_strides_extra = numel(t_vals);
        Data.(AbrevFileNames(i)).num_strides = num_strides_extra - 1;

        % Make sure not ending too soon, add end time?
        for k = 1:num_strides_extra -1

            curX = Data.(AbrevFileNames(i)).x(t_vals_idx(k):t_vals_idx(k+1));
            curT = Data.(AbrevFileNames(i)).t(t_vals_idx(k):t_vals_idx(k+1));

            % Normalizing each stride
            curX = curX - curX(1);
            curT = curT - curT(1);

            Data.(AbrevFileNames(i)).(strcat("Stride_", num2str(k))).x = curX;
            Data.(AbrevFileNames(i)).(strcat("Stride_", num2str(k))).t = curT;

        end

    end

end
