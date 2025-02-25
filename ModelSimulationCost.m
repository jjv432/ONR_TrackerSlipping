function [Cost] = ModelSimulationCost(freeParams)
    global cur_x_vals cur_t_vals

    % Run the simulation to determine the range of x values
    [simulation_x_array, simulation_time] = UnifiedModelLight(freeParams);

    % Determine the cost of this simulation by using RSS

    persistent p 
    
    if isempty(p)
        p = polyfit(cur_t_vals, cur_x_vals, 5);
    end

    data_spline = polyval(p, simulation_x_array);

    Cost = sum((simulation_x_array - data_spline).^2);
end
  
