clc; clearvars -except Data; close all; format compact;

addpath("SlippingFunctions");
addpath("TrackerFiles");
addpath("Figures");
%{

TO-DO

Need to re-write this so that multiple trials of the same frequency can be
used.  Can concatenate in the "Sorting each trial by stride based on time
alone" section.  Also in this section, ensure correct number of plots are
being made (comment is in there with more detail)

Eventually, move sections to their own functions

There's somthing about the 0p5 Hz trial that is causing stride 4 to have
one less index than all of the other strides... curently have a not so
great fix for it

Add more helpful titles- base it off frequency/ file name

Combine functions where they don't need to be seperate.

Add in a way to save the data struct- maybe don't call a few functions when
data struct can be loaded instead

Make a switch statement based on the behavior that the use desires?

%}

%% Asking user if they want to read from a .json
% Change this so that it allows user to enter the name
readFromJson = input("Do you want to read Data from a .mat named Data?", 's');
if readFromJson == 'y'
    clear Data
    Data = load("Data.mat");

end

%% Only doing some functions if Data doesn't exist in the workspace

if ~exist('Data', 'var')
    [Data, FullFileNames, AbrevFileNames] = makeNewDataStruct();
    Data.FullFileNames = FullFileNames;
    Data.AbrevFileNames = AbrevFileNames;
else
    FullFileNames = Data.FullFileNames;
    AbrevFileNames = Data.AbrevFileNames;

end


%% Plotting the data so that you can see each stride

%{
A trial exists BETWEEN two vertical lines. 
%}
for i = 1:numel(FullFileNames)
    close all
    figure()
    hold on

    plot(Data.(AbrevFileNames(i)).t, Data.(AbrevFileNames(i)).x);
    accumulated_time = Data.(AbrevFileNames(i)).t(Data.(AbrevFileNames(i)).StartIndex);
    xline(accumulated_time);
    for k = 1:Data.(AbrevFileNames(i)).num_strides

        cur_end_time = Data.(AbrevFileNames(i)).(strcat("Stride_", num2str(k))).t(end);

        xline(cur_end_time + accumulated_time);

        accumulated_time = accumulated_time + cur_end_time;

    end
    hold off

    remove_answer = input("Want to remove any of the trials? (y/n)", 's');

    if remove_answer == 'y'
        num_removal_answer = input("How many trials, starting from the end, should be removed?");
        TempData = Data;
        looks_bad = 1;
        close

        while (looks_bad == 1)
            figure()
            hold on

            plot(TempData.(AbrevFileNames(i)).t, TempData.(AbrevFileNames(i)).x);
            accumulated_time = TempData.(AbrevFileNames(i)).t(TempData.(AbrevFileNames(i)).StartIndex);
            xline(accumulated_time);
            for a = 1:TempData.(AbrevFileNames(i)).num_strides - num_removal_answer

                cur_end_time = TempData.(AbrevFileNames(i)).(strcat("Stride_", num2str(a))).t(end);

                xline(cur_end_time + accumulated_time);

                accumulated_time = accumulated_time + cur_end_time;

            end
            hold off
            looking_good = input("Does this look correct? (y/n)", 's');
            if looking_good == 'y'
                looks_bad = 0;
                % TempData.(AbrevFileNames(i)).num_strides = TempData.(AbrevFileNames(i)).num_strides - num_removal_answer - 1;
            end

        end

        % Need to remove fields from the struct here
        TempFrequencyField = TempData.(AbrevFileNames(i));
        for b = TempData.(AbrevFileNames(i)).num_strides:-1:(TempData.(AbrevFileNames(i)).num_strides- num_removal_answer)

            TempFrequencyField = rmfield(TempFrequencyField, strcat("Stride_", num2str(b)));
        end

        TempData.(AbrevFileNames(i)) = TempFrequencyField;
        TempData.(AbrevFileNames(i)).num_strides = (TempData.(AbrevFileNames(i)).num_strides- num_removal_answer - 1);
        Data = TempData;
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
        cur_x = Data.(AbrevFileNames(i)).(strcat("Stride_", num2str(k))).x';

        % Horrible fix to this, but not sure how else to fix this issue
        if numel(cur_x) < size(x_tot, 2)
            cur_x = [cur_x, cur_x(end)];
        end
      
       
        % This works because we're assuming dt is constant
        x_tot = [x_tot; cur_x];

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
    title(strcat("Normalized X-Position for ", num2str(Data.(AbrevFileNames(i)).frequency), "Hz"));
    hold off
    ylim([-.1 .2]);
    saveas(gcf, strcat("Figures/", num2str(Data.(AbrevFileNames(i)).frequency), '.fig'));


    clear max_x_vals min_x_vals

end


%% Asking the user if they want to save these results

save_answer = input("Want to save these results to a .mat file? (y/n)", 's');

if save_answer == 'y'
    save("Data.mat", '-struct', 'Data');
end
