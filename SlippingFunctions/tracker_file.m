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

            num_strides_extra = numel(t_vals);
            obj.StrideCount = num_strides_extra - 1;

            % Make sure not ending too soon, add end time?
            for k = 1:num_strides_extra -1

                curX = obj.x(t_vals_idx(k):t_vals_idx(k+1));
                curT = obj.t(t_vals_idx(k):t_vals_idx(k+1));

                % Normalizing each stride
                curX = curX - curX(1);
                curT = curT - curT(1);

                obj.Strides(k).x = curX;
                obj.Strides(k).t = curT;
                obj.Strides(k).Index = t_vals_idx(k);

            end

        end

        function PlotStrides(obj)

            close all
            figure()
            hold on

            plot(obj.t, obj.x);
            title(obj.FileName);
            accumulated_time = obj.t(obj.StartingIndex);
            xline(accumulated_time);
            for k = obj.StrideCount:-1:1

                cur_end_time = obj.Strides(k).t(end);

                xline(cur_end_time + accumulated_time);

                accumulated_time = accumulated_time + cur_end_time;

            end
            hold off



        end

        function AdjustStrides(obj)

            doneBool = 0;
            close all
            obj.PlotStrides();

            while(~doneBool)

                desired_stride = input("Counting from RIGHT TO LEFT, which stride do you want to adjust?\n");
                operation_type = input("Will this be deletion (d) or moving (m)?", 's');
                pause();

                switch(operation_type)
                    case 'd'
                        obj.Strides(:, desired_stride) = [];
                        obj.StrideCount = obj.StrideCount - 1;
                    case 'm'
                        %\cite(https://www.mathworks.com/matlabcentral/answers/101415-how-to-get-input-from-user-without-asking-to-press-enter#answer_110763
                        p = '';

                        while p~= 'x'
                            disp("In while loop");
                            pause();
                            w = waitforbuttonpress;

                            if w
                                fprintf("W is 1");
                                p = get(gcf, 'CurrentCharacter');
                                disp(p);

                                switch(p)
                                    case 'a'
                                        obj.Strides(desired_stride).Index = obj.Strides(desired_stride).Index - 1;

                                    case 'd'
                                        obj.Strides(desired_stride).Index = obj.Strides(desired_stride).Index + 1;

                                end
                                obj.PlotStrides();
                                disp(p)
                            end
                        end

                end
                close
                obj.PlotStrides();

                doneBool = input("Are you done? (1/0)");

            end


        end
    end
end