function [virtual_goal, status] = calculate_virtual_goal(agent, params)
    % FINAL, DEFINITIVE VERSION with STABILIZED Tangential Navigation

    virtual_goal = agent.virtual_goal;
    status = agent.status;

    switch agent.status
        case 0
            virtual_goal = params.goal;
            
        case 1 % Tangential Navigation
            if ~isempty(agent.sensed_obstacles)
                % --- STABILIZATION LOGIC ---
                % Find the single closest point for reference
                dists = vecnorm(agent.sensed_obstacles - agent.pos);
                [~, min_idx] = min(dists);
                closest_obs_pt = agent.sensed_obstacles(:, min_idx);
                
                vec_to_goal = params.goal - agent.pos; 
                vec_to_closest_obs = closest_obs_pt - agent.pos;
                
                % Calculate angles relative to the closest point
                alpha_i = atan2(vec_to_goal(2), vec_to_goal(1)); 
                beta_i = atan2(vec_to_closest_obs(2), vec_to_closest_obs(1));

                % --- Decide turn direction based on Beta ---
                % This core logic from the paper determines if we follow left or right
                if beta_i >= 0 % Obstacle is generally to the left
                    gamma_i = beta_i - alpha_i - pi/2; 
                else % Obstacle is generally to the right
                    gamma_i = beta_i - alpha_i + pi/2; 
                end

                % Apply rotation to find the stable tangential direction
                R = [cos(gamma_i), -sin(gamma_i); sin(gamma_i), cos(gamma_i)];
                virtual_goal = agent.pos + R * vec_to_goal;
            else
                virtual_goal = agent.pos; 
            end

        case 2 % Endpoint Maneuver
            vec_to_goal = params.goal - agent.pos;
            alpha_i = atan2(vec_to_goal(2), vec_to_goal(1));
            if abs(alpha_i) < params.delta_turn * 2 
                status = 0; 
                virtual_goal = params.goal;
            else
                % Use the stored angle from the last obstacle contact
                if isempty(agent.last_obstacle_angle), agent.last_obstacle_angle = 0; end % Safety check
                
                if agent.last_obstacle_angle >= 0, gamma_i = params.delta_turn; else, gamma_i = -params.delta_turn; end
                R = [cos(gamma_i), -sin(gamma_i); sin(gamma_i), cos(gamma_i)];
                
                % Use the stored last obstacle point as pivot
                if isempty(agent.last_obstacle_point), agent.last_obstacle_point = agent.pos; end % Safety check
                
                vec_pivot_to_agent = agent.pos - agent.last_obstacle_point;
                new_vec_from_pivot = R * vec_pivot_to_agent;
                virtual_goal = agent.last_obstacle_point + new_vec_from_pivot;
            end
            
        case 3 % Corner Avoidance
            if size(agent.sensed_obstacles, 2) >= 2
                avg_obs_point = mean(agent.sensed_obstacles(:,1:2), 2);
                away_vec = agent.pos - avg_obs_point;
                 if norm(away_vec) < 1e-6, away_vec = randn(2,1); end % Avoid zero vector
                escape_distance = params.rs * 0.75;
                virtual_goal = agent.pos + (away_vec / norm(away_vec)) * escape_distance;
            else
                virtual_goal = agent.pos;
            end
    end
end