%% Function for chopping off data at a given time

function chop_index = chop_data(unchopped_data, end_time)

    chop_index = find(unchopped_data.t >= end_time, 1, 'first');


end

