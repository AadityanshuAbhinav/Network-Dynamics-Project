function plot_simulation_state(agents, obstacles, goal, t, comm_radius)
    clf; hold on; axis equal; grid on;
    status_colors = {'b', 'g', 'm', 'c', 'y', 'k'}; 
    
    for i = 1:length(obstacles)
        plot(obstacles{i}(1,:), obstacles{i}(2,:), 'k-', 'LineWidth', 3);
    end
    
    plot(goal(1), goal(2), 'r*', 'MarkerSize', 12, 'LineWidth', 2);
    
    for i = 1:length(agents)
        for j = (i+1):length(agents)
            if norm(agents(i).pos - agents(j).pos) < comm_radius
                line([agents(i).pos(1) agents(j).pos(1)], [agents(i).pos(2) agents(j).pos(2)], ...
                    'Color', [0.8 0.8 0.8], 'LineStyle', '--');
            end
        end
    end
    
    for i = 1:length(agents)
        pos = agents(i).pos; vel = agents(i).vel;
        status_idx = agents(i).status + 1;
        if status_idx > length(status_colors), status_idx = length(status_colors); end
        color = status_colors{status_idx};
        
        if norm(vel) > 0.1, heading = atan2(vel(2), vel(1)); else, heading = 0; end
        sz = 5.0;
        p1 = pos + sz*[cos(heading); sin(heading)];
        p2 = pos + sz*0.8*[cos(heading+2.5); sin(heading+2.5)];
        p3 = pos + sz*0.8*[cos(heading-2.5); sin(heading-2.5)];
        patch([p1(1) p2(1) p3(1)], [p1(2) p2(2) p3(2)], color);
    end
    
    xlim([-20 180]); ylim([-80 150]);
    title(sprintf('Collective Navigation Simulation (t = %.2f s)', t));
    xlabel('X-Position'); ylabel('Y-Position');
    legend({'Obstacle', 'Goal'}, 'Location', 'northeast');
end