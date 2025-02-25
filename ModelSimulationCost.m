function [Cost] = ModelSimulationCost(freeParams)
    global cur_x_vals

    % Run the simulation to determine the range of x values
    [simulation_x_array, simulation_time] = UnifiedModelLight(freeParams);

    % Determine the cost of this simulation by using RSS
    % appended_experimental_x_vals = cur_x_vals(1:numel(simulation_x_array));

    xq = linspace(simulation_time(1), simulation_time(end), numel(cur_x_vals));

    vq1 = spline(simulation_time, simulation_x_array, xq);

    % figure()
    % plot(xq, cur_x_vals); hold on; plot(xq, vq1, 'o')

    Cost = sum((cur_x_vals - vq1').^2);
end
  
