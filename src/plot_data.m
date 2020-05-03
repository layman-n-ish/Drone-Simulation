% Plots the simulated run
function plot_data(data)
    figure; plots = [subplot(3, 2, 1), subplot(3, 2, 2), subplot(3, 2, 3), subplot(3, 2, 4), subplot(3, 2, 5:6)]; 
         
    subplot(plots(1));
    multiplot(data, data.x);
    xlabel('Time (s)');
    ylabel('Linear Displacement (m)');
    title('Linear Displacement');

    subplot(plots(2));
    multiplot(data, data.vel);
    xlabel('Time (s)');
    ylabel('Linear Velocity (m/s)');
    title('Linear Velocity');

    subplot(plots(3));
    multiplot(data, rad2deg(data.theta));
    xlabel('Time (s)');
    ylabel('Angular Displacement (deg)');
    title('Angular Displacement');

    subplot(plots(4));
    multiplot(data, rad2deg(data.angvel));
    xlabel('Time (s)');
    ylabel('Angular Velocity (deg/s)');
    title('Angular Velocity')

    subplot(plots(5));
    plot(data.t, sqrt(data.input(1, :)), 'r-', data.t, sqrt(data.input(2, :)), 'g.', ...
    data.t, sqrt(data.input(3, :)), 'b.', data.t, sqrt(data.input(4, :)), 'y');
    xlabel('Time (s)');
    ylabel('Angular Velocity (rad/s)');
    title('Control Inputs (motor speed)')
end

% Plot three components of a vector in RGB.
function multiplot(data, values)
    % Select the parts of the data to plot.
    times = data.t(:, 1:end);
    values = values(:, 1:end);

    % Plot in RGB, with different markers for different components.
    plot(times, values(1, :), 'r-', times, values(2, :), 'g.', times, values(3, :), 'b.');
    
    % Set axes to remain constant throughout plotting.
    xmin = min(data.t);
    xmax = max(data.t);
    ymin = 1.1 * min(min(values));
    ymax = 1.1 * max(max(values)) + 1;
    axis([xmin xmax ymin ymax]);
end