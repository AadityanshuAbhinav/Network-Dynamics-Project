function agent = update_backmost_agent(agent, neighbors)
    % Implements the gossip algorithm (Eq. 54) to identify the backmost agent.
    % This is a simplified version capturing the core logic.

    % The paper defines a new coordinate system based on the endpoint geometry.
    % We'll simplify by using the vector from the agent to the goal as the
    % primary axis of motion.
    
    if isempty(agent.backmost_agent_id)
        agent.backmost_agent_id = agent.id;
    end

    % Define the axis of motion (e.g., from agent's virtual goal, which would be the endpoint)
    % For simplicity, we assume the general motion is along the x-axis for this scenario.
    axis_of_motion = [1; 0]; 

    % Get the projected position of the agent's current backmost candidate
    backmost_agent_pos = agent.pos; % Assume current agent if no better info
    if ~isempty(neighbors)
        % Find the position of the agent believed to be backmost
        candidate_ids = [neighbors.id];
        backmost_idx_in_neighbors = find(candidate_ids == agent.backmost_agent_id, 1);
        if ~isempty(backmost_idx_in_neighbors)
            backmost_agent_pos = neighbors(backmost_idx_in_neighbors).pos;
        end
    end
    
    my_backmost_projection = backmost_agent_pos' * axis_of_motion;
    
    % Check neighbors for a "more back" agent
    for i = 1:length(neighbors)
        neighbor = neighbors(i);
        
        % Get the neighbor's idea of who is backmost
        if isempty(neighbor.backmost_agent_id)
            neighbor_backmost_id = neighbor.id;
            neighbor_backmost_pos = neighbor.pos;
        else
            neighbor_backmost_id = neighbor.backmost_agent_id;
            % Find position of this agent
            all_neighbor_ids = [neighbors.id];
            pos_idx = find(all_neighbor_ids == neighbor_backmost_id, 1);
            if ~isempty(pos_idx)
                neighbor_backmost_pos = neighbors(pos_idx).pos;
            else % If not in my neighborhood, I can't evaluate it
                continue;
            end
        end
        
        neighbor_projection = neighbor_backmost_pos' * axis_of_motion;
        
        % If the neighbor's candidate is further behind mine, adopt it
        if neighbor_projection < my_backmost_projection
            my_backmost_projection = neighbor_projection;
            agent.backmost_agent_id = neighbor_backmost_id;
        end
    end
end