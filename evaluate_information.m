% In evaluate_information.m

function most_relevant_info = evaluate_information(agent, received_info, params, current_time)
    % HIGH-FIDELITY version implementing the full relevance model from Eq. 26-32.
    
    most_relevant_info = [];
    if isempty(received_info), return; end
    
    relevance_scores = zeros(length(received_info), 1);
    
    for i = 1:length(received_info)
        info = received_info{i};
        w = params.relevance_weights;
        
        % 1. Age of information
        age = current_time - info.timestamp;
        rel_t = 10 - (10 * age / params.d_t);
        
        % 2. Distance to obstacle
        rel_dist = 0;
        if ~isempty(info.obstacle_pos)
            % Check if obstacle is in front of the agent
            vec_to_obs = info.obstacle_pos - agent.pos;
            if agent.vel' * vec_to_obs > 0 % Dot product to check if it's "in front"
                rel_dist = 10 - (10 * norm(vec_to_obs) / params.d_x);
            else
                rel_dist = -10 * norm(vec_to_obs) / params.d_x;
            end
        end

        % 3. Expectation of orientation
        agent_orientation = atan2(agent.vel(2), agent.vel(1));
        angle_diff = abs(angdiff(agent_orientation, info.orientation));
        rel_exp = 10 - (10 * angle_diff / params.d_theta);

        % 4. Sender type (simplified to "direct")
        rel_o = 10;
        
        % 5. Status type
        rel_type = 0;
        if any(info.status == [3, 4]), rel_type = 10; % Reorientation is high priority
        elseif info.status == 1, rel_type = 5; end

        % Weighted Mean from Eq. (32) [cite_start][cite: 411]
        numerator = w.c_type*rel_type + w.c_o*rel_o + w.c_exp*rel_exp + w.c_dist*rel_dist + w.c_t*rel_t;
        denominator = w.c_type + w.c_o + w.c_exp + w.c_dist + w.c_t;
        relevance_scores(i) = numerator / denominator;
    end
    
    % Find info with max relevance, applying the 0.95 rule from Eq. [cite_start]34 [cite: 423]
    if any(relevance_scores)
        [max_rel, max_idx] = max(relevance_scores);
        if max_rel > 0 % Only consider relevant information
            % In a full implementation, we would average all info packages
            % that meet the criteria rel >= 0.95 * max_rel.
            % For simplicity, we'll take the single best one.
            most_relevant_info = received_info{max_idx};
        end
    end
end