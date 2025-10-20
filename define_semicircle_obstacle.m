function vertices = define_semicircle_obstacle(center_x, center_y, radius)
    % Creates a polygon to approximate a semi-circle for collision detection.
    % Defines the arc on the right side (positive x relative to center).
    
    theta = linspace(-pi/2, pi/2, 30); % Create 30 points for the arc
    x = radius * cos(theta) + center_x;
    y = radius * sin(theta) + center_y;
    
    % Close the semi-circle with a straight line back to the start
    vertices = [x, x(1); y, y(1)]; 
end