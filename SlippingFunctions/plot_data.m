%% Function for plotting data
function plot_data(data, i, AbrevFileNames, neg_dx_indices, chop_index)

    figure()
    hold on
    xlim([0, data.t(chop_index)]);
    plot(data.t(1:chop_index), data.x(1:chop_index), "LineWidth",3.5);
    ylabel("X-Position (m)")
    xlabel("Time (s)")
    title(strcat("X Position for ", AbrevFileNames(i)))
    scatter(data.t(neg_dx_indices), data.x(neg_dx_indices), 75, 'xr', 'LineWidth',1.5);
    legend("Foot Position", "Slippage Detected", 'location', 'northwest');
    grid on;
    hold off
    
end