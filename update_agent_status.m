function agent = update_agent_status(agent, params)
    % FINAL, DEFINITIVE AND CORRECTED VERSION
    
    sensed_obs_count = size(agent.sensed_obstacles, 2);
    new_status = agent.status;

    if sensed_obs_count >= 2
        % This is the critical fix: When multiple points are seen, the primary
        % strategy should be to commit to following the CLOSEST one tangentially,
        % not to assume it's a corner. This prevents the channel-corner confusion.
        new_status = 1; % Prioritize Tangential Navigation

    elseif sensed_obs_count == 1
        new_status = 1; % Standard Tangential Navigation

    else % No obstacles are sensed
        if agent.saw_obstacle_last_frame
            new_status = 2; % We just lost an obstacle, trigger Endpoint Maneuver
            if ~isempty(agent.last_obstacle_point_history)
                 agent.last_obstacle_point = agent.last_obstacle_point_history;
                 vec_to_obs = agent.last_obstacle_point - agent.pos;
                 agent.last_obstacle_angle = atan2(vec_to_obs(2), vec_to_obs(1));
            end
        else
            new_status = 0; % Path is clear, move to goal
        end
    end
    
    if ~isempty(agent.sensed_obstacles)
        % Find the closest obstacle point to store for endpoint maneuvers
        dists = vecnorm(agent.sensed_obstacles - agent.pos);
        [~, min_idx] = min(dists);
        agent.last_obstacle_point_history = agent.sensed_obstacles(:, min_idx);
    end
    
    agent.status = new_status;
end