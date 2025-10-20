function vertices = define_zigzag_obstacle()
    % Defines the vertices for the zigzag obstacle from Figure 13.
    % CORRECTED: This function now correctly returns a matrix, not a cell array.
    
    vertices = [
        50, 50, 80, 50, 80, 50, 50;  % X-coordinates
       -20, 20, 50, 80, 110, 120, 140 % Y-coordinates
    ];

    % The main script is responsible for putting this matrix into a cell.
end