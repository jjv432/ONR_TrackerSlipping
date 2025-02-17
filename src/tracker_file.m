classdef tracker_file < handle
    %UNTITLED2 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        t
        x
        y
        r
        vx
        vy
        v
        ax
        ay
        a
        FileName
        NominalGaitFrequency
        StartingIndex
        HSIndices
        StrideTimeValues
        StrideCount
        Strides
        MeanXPosition
        StdDevXPosition
        MinXPosition
        MaxXPosition
        StatsPlottingTrialLength

    end

    methods
        function obj = tracker_file(txtfile)

            if nargin == 1
                temp = readmatrix(txtfile);

                cur_col_size = size(temp, 2);

                % Sort file into each column

                if cur_col_size == 7
                    obj.t = temp(:, 1)- temp(1,1);
                    obj.x = temp(:, 2)- temp(1,2);
                    obj.y = temp(:, 3)- temp(1,3);
                    obj.vx = temp(:, 4);
                    obj.vy = temp(:, 5);
                    obj.ax = temp(:, 6);
                    obj.ay = temp(:, 7);

                elseif cur_col_size == 10
                    obj.t = temp(:, 1)- temp(1,1);
                    obj.x = temp(:, 2)- temp(1,2);
                    obj.y = temp(:, 3)- temp(1,3);
                    obj.r = temp(:, 4);
                    obj.vx = temp(:, 5);
                    obj.vy = temp(:, 6);
                    obj.v = temp(:, 7);
                    obj.ax = temp(:, 8);
                    obj.ay = temp(:, 9);
                    obj.a = temp(:, 10);
                end

                if contains(string(txtfile), "_1p0Hz")
                    obj.NominalGaitFrequency = 1.0;
                elseif contains(string(txtfile), "_2p5Hz")
                    obj.NominalGaitFrequency = 2.5;
                elseif contains(string(txtfile), "_4p0Hz")
                    obj.NominalGaitFrequency = 4.0;
                elseif contains(string(txtfile), "_0p5Hz")
                    obj.NominalGaitFrequency = 0.5; % using fft for orignal file
                end

            end
        end

        function PickStartingPoint(obj)
            % Referenced from: https://github.com/jjv432/ONR_Friction_Calcs/blob/main/frameParser.m
            fig = figure;
            fig.WindowState = "Maximized";
            plot(obj.t, obj.y)
            xlabel("Time (s)");
            ylabel("X-Position (x)");
            title("X v T for " + obj.FileName);

            datacursormode on
            dcm_obj = datacursormode(fig);

            fprintf("Select beinning point of the data\n");
            pause
            % Export cursor to workspace
            info_struct = getCursorInfo(dcm_obj);

            %Putting variables in the order of trials
            obj.StartingIndex = info_struct.DataIndex;

            close
        end

        function CreateStridePredicitons(obj)


            t_not = obj.t(obj.StartingIndex);
            t_end = obj.t(end);
            dt = 1/obj.NominalGaitFrequency;
            t_vals = t_not:dt:t_end;


            %\cite{https://www.mathworks.com/matlabcentral/answers/152301-find-closest-value-in-array#comment_2806253}
            [~, t_vals_idx] = min(abs(obj.t - t_vals));
            obj.StrideTimeValues = t_vals;
            obj.HSIndices = t_vals_idx;

            obj.UpdateStridePositions;

        end

        function UpdateStridePositions(obj)
            num_strides_extra = numel(obj.StrideTimeValues);
            obj.StrideCount = num_strides_extra - 1;

            % Make sure not ending too soon, add end time?
            for k = 1:num_strides_extra -1

                curX = obj.x(obj.HSIndices(k):obj.HSIndices(k+1));
                curT = obj.t(obj.HSIndices(k):obj.HSIndices(k+1));

                % Normalizing each stride
                curX = curX - curX(1);
                curT = curT - curT(1);

                obj.Strides(k).x = curX;
                obj.Strides(k).t = curT;
                obj.Strides(k).Indices = [obj.HSIndices(k),obj.HSIndices(k+1)];

            end

        end

        function PlotStrides(obj)

            clf;
            axs = gca;


            hold on

            plot(axs, obj.t, obj.x);
            title(obj.FileName);

            for k = 1:obj.StrideCount

                xline(obj.t(obj.Strides(k).Indices(1)))
                xline(obj.t(obj.Strides(k).Indices(2)))

            end
            hold off



        end

        function AdjustStrides(obj)

            doneBool = 0;
            close all
            obj.PlotStrides();

            while(~doneBool)

                desired_stride = input("Counting from LEFT TO RIGHT, which line do you want to adjust?\n");
                operation_type = input("Will this be deletion (d) or moving (m)?", 's');


                switch(operation_type)
                    case 'd'
                        obj.Strides(:, desired_stride) = [];
                        obj.StrideCount = obj.StrideCount - 1;
                    case 'm'
                        fprintf("Use 'a' to move left, 's' to move right, 'x' to exit")

                        p = "";
                        while(p~='x')
                            w = waitforbuttonpress;
                            if w
                                p = get(gcf, 'CurrentCharacter');
                                p = string(p);
                                disp(p);

                                switch(p)
                                    case 'a'
                                        obj.HSIndices(desired_stride)= obj.HSIndices(desired_stride) -1;

                                    case 's'
                                        obj.HSIndices(desired_stride)= obj.HSIndices(desired_stride) + 1;
                                end

                                obj.UpdateStridePositions
                                if desired_stride == obj.StrideCount
                                    obj.StartingIndex = newIndex;
                                end
                                obj.PlotStrides();
                                disp(p)
                            end


                        end

                end
                obj.PlotStrides();

                doneBool = input("Are you done? (1/0)");

            end


        end

        function GenerateStatistics(obj)
            ShortestTrialLength = inf;
            for k = 1:obj.StrideCount
                delta = obj.Strides(k).Indices(2) - obj.Strides(k).Indices(1);

                if delta < ShortestTrialLength
                    ShortestTrialLength = delta;
                end

            end
            obj.StatsPlottingTrialLength = ShortestTrialLength;

            x_matrix = [];
            for k = obj.StrideCount:-1:1
                x_matrix(:, k) = obj.Strides(k).x(1:ShortestTrialLength);

            end

            obj.MeanXPosition  = mean(x_matrix, 2);
            obj.StdDevXPosition = std(x_matrix, [], 2);
            obj.MaxXPosition = max(x_matrix, [], 2);
            obj.MinXPosition = min(x_matrix, [], 2);

        end
        function PlotStatistics(obj)
            figure();
            hold on
            fill([obj.t(1:obj.StatsPlottingTrialLength); flip(obj.t(1:obj.StatsPlottingTrialLength))], [(obj.MeanXPosition - obj.StdDevXPosition); flip(obj.MeanXPosition + obj.StdDevXPosition)], [0.8 0.8 0.8]); % \cite{https://www.mathworks.com/matlabcentral/answers/1928100-create-plot-with-shaded-standard-deviation-but#answer_1192320}
            plot(obj.t(1:obj.StatsPlottingTrialLength), obj.MeanXPosition, '--k');
            plot(obj.t(1:obj.StatsPlottingTrialLength), obj.MaxXPosition, '--b');
            plot(obj.t(1:obj.StatsPlottingTrialLength), obj.MinXPosition, '--r');
            legend('', "Mean", "Max", "Min");
            xlabel("Time(s)");
            ylabel("Change in X-Position(m)");
            title(string(obj.FileName))
            ylim([-.1 .2]);
            hold off
        end

    end
end