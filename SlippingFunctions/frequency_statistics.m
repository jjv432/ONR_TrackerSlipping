function Data = frequency_statistics(Data, FullFileNames, AbrevFileNames)


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
end