classdef tracker_file
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

        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end