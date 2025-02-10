function Data = trial_finder(Data, FullFileNames, AbrevFileNames)


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

end