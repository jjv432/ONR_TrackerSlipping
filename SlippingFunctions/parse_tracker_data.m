%% Function for sorting data

function data = parse_tracker_data(input_file)

    % Read file
    temp = readmatrix(input_file);

    cur_col_size = size(temp, 2);

    % Sort file into each column
    %{
For whatever reason, not all of the files had the same number of columns.
This is a quick and dirty solution to this.  The small ones don't have
columns for the following values: r, v, a.  
    %}

    if cur_col_size == 7
        data.t = temp(:, 1)- temp(1,1);
        data.x = temp(:, 2)- temp(1,2);
        data.y = temp(:, 3)- temp(1,3);        
        data.vx = temp(:, 4);
        data.vy = temp(:, 5);
        data.ax = temp(:, 6);
        data.ay = temp(:, 7);

    elseif cur_col_size == 10
        data.t = temp(:, 1)- temp(1,1);
        data.x = temp(:, 2)- temp(1,2);
        data.y = temp(:, 3)- temp(1,3);
        data.r = temp(:, 4);
        data.vx = temp(:, 5);
        data.vy = temp(:, 6);
        data.v = temp(:, 7);
        data.ax = temp(:, 8);
        data.ay = temp(:, 9);
        data.a = temp(:, 10);
    end
end