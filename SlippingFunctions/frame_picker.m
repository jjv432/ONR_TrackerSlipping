function Frames = frame_picker(plotting_data, trial_name)

    % Referenced from: https://github.com/jjv432/ONR_Friction_Calcs/blob/main/frameParser.m
    fig = figure;
    fig.WindowState = "Maximized";
    plot(plotting_data.t, plotting_data.y)
    xlabel("Time (s)");
    ylabel("X-Position (x)");
    title("X v T for " + trial_name);

    datacursormode on
    dcm_obj = datacursormode(fig);

    fprintf('Select beinning point\n');
    pause
    % Export cursor to workspace
    info_struct = getCursorInfo(dcm_obj);

    %% Organizing User Input

    [~, doubleTrials] = size(info_struct);
    
    %Storing a list of all of the frame numbers
    Frames = [];
    for i = 1:doubleTrials

        Frames = [Frames; info_struct(i).DataIndex];

    end
    
    %Putting variables in the order of trials 
    Frames = sort(Frames, 1);

end