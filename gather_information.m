% In gather_information.m

function received_info = gather_information(current_agent_idx, all_agents, neighbor_indices, current_time)
    % UPDATED to create structured info packages with timestamps and orientation.
    
    received_info = {};
    if isempty(neighbor_indices)
        return;
    end
    
    for i = 1:length(neighbor_indices)
        neighbor = all_agents(neighbor_indices(i));
        
        package.sender_id = neighbor.id;
        package.status = neighbor.status;
        package.pos = neighbor.pos;
        package.vel = neighbor.vel;
        
        % Add orientation (angle of velocity vector)
        package.orientation = atan2(neighbor.vel(2), neighbor.vel(1));
        
        % Add timestamp for age evaluation
        package.timestamp = current_time;
        
        % Add obstacle info if available (needed for rel_dist)
        if ~isempty(neighbor.sensed_obstacles)
            package.obstacle_pos = neighbor.sensed_obstacles(:,1);
        else
            package.obstacle_pos = [];
        end
        
        received_info{end+1} = package;
    end
end