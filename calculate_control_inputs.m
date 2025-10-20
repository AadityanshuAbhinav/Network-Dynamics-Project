function u_total = calculate_control_inputs(agent, neighbors, params)
    % FINAL, COMPLETE AND VERIFIED VERSION
    
    u_alpha_gradient = [0; 0]; u_alpha_consensus = [0; 0];
    d_sigma = sigma_norm(params.d_alpha, params.epsilon);
    r_sigma = sigma_norm(params.rc, params.epsilon);

    for i = 1:length(neighbors)
        nij = neighbors(i).pos - agent.pos;
        dist_sigma = sigma_norm(nij, params.epsilon);
        phi_alpha = rho_h(dist_sigma / r_sigma, params.h_alpha) * (dist_sigma - d_sigma);
        u_alpha_gradient = u_alpha_gradient + params.c1_alpha * phi_alpha * (nij / sqrt(1 + params.epsilon * norm(nij)^2));
        u_alpha_consensus = u_alpha_consensus + params.c2_alpha * rho_h(dist_sigma / r_sigma, params.h_alpha) * (neighbors(i).vel - agent.vel);
    end
    u_alpha = u_alpha_gradient + u_alpha_consensus;

    u_beta = [0; 0];
    if ~isempty(agent.sensed_obstacles)
        for i = 1:size(agent.sensed_obstacles, 2)
            dir_vec = agent.pos - agent.sensed_obstacles(:, i);
            dist = norm(dir_vec);
            if dist > 0, u_beta = u_beta + params.c1_beta * (1/dist - 1/params.rs) * (dir_vec / dist); end
        end
    end
    
    if agent.status == 2 % Endpoint turning controller
        v_ref = (agent.virtual_goal - agent.pos) / norm(agent.virtual_goal - agent.pos) * params.v_max * 0.7;
        u_gamma = 1.5 * (v_ref - agent.vel); 
    else % Standard velocity-tracking controller
        dir_to_goal = agent.virtual_goal - agent.pos;
        if norm(dir_to_goal) > 1e-6
            desired_vel = (dir_to_goal / norm(dir_to_goal)) * params.v_max;
            if norm(dir_to_goal) < 5, desired_vel = desired_vel * (norm(dir_to_goal) / 5); end
            u_gamma = params.c1_gamma * (desired_vel - agent.vel);
        else, u_gamma = -params.c2_gamma * agent.vel; end
    end

    u_total = u_alpha + u_beta + u_gamma;
end