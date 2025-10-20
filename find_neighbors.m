function neighbor_indices = find_neighbors(current_agent_idx, all_agents, comm_radius)
    % Finds the indices of all agents within the communication radius of the current agent.
    % This models the neighborhood definition from the paper.
    
    neighbor_indices = [];
    current_agent_pos = all_agents(current_agent_idx).pos;
    num_agents = length(all_agents);
    
    for i = 1:num_agents
        % An agent cannot be its own neighbor
        if i == current_agent_idx
            continue;
        end
        
        % Calculate Euclidean distance to the other agent
        dist = norm(current_agent_pos - all_agents(i).pos);
        
        % If within communication range, add its index to the list
        if dist < comm_radius
            neighbor_indices = [neighbor_indices, i];
        end
    end
end