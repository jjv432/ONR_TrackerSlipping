
%% Function for plotting one gait cycle

function plot_gait(data, i, AbrevFileNames, neg_dx_indices, beginningIndex, endIndex)
    figure()
    hold on
    xlim([data.t(beginningIndex), data.t(endIndex)]);  
    plot(data.t(beginningIndex:endIndex), data.x(beginningIndex:endIndex), "LineWidth",3.5);
    ylabel("X-Position (m)")
    xlabel("Time (s)")
    title(strcat("X Position for ", AbrevFileNames(i)))
    % scatter(data.t(neg_dx_indices), data.x(neg_dx_indices), 75, 'xr', 'LineWidth',1.5);
    legend("Foot Position", "Slippage Detected", 'location', 'northwest');
    grid on;
    hold off


end
