function vertices = define_circle_obstacle(center_x, center_y, radius)
    % Creates a polygon to approximate a circle for collision detection.
    
    theta = linspace(0, 2*pi, 30); % Create 30 points for a smooth circle
    x = radius * cos(theta) + center_x;
    y = radius * sin(theta) + center_y;
    vertices = [x; y];
end