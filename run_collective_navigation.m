% run_collective_navigation.m
% FINAL DEFINITIVE VERSION

clear; clc; close all;

%% Scenario Selector
scenario = 'semicircle'; % Options: 'zigzag', 'two_circles', 'semicircle'

%% Simulation Parameters
params.N = 12; 
params.dt = 0.02;
params.t_max = 220;

% Final, Balanced Control Gains
params.c1_alpha = 10; params.c2_alpha = 2 * sqrt(10);
params.c1_beta = 50; params.c2_beta = 2 * sqrt(50);
params.c1_gamma = 3.0; 
params.c2_gamma = 0.2;

% Other necessary parameters
params.rs = 15; params.rc = 20; params.v_max = 4;
params.d_alpha = 7; params.epsilon = 0.1; params.h_alpha = 0.2;
params.delta_turn = deg2rad(15);

%% Initialization
% (Agent initialization remains the same)
agents(params.N) = struct();
for i = 1:params.N
    agents(i).id = i;
    agents(i).pos = rand(2, 1) * 20 - 10;
    agents(i).vel = [0; 0]; agents(i).status = 0;
    agents(i).virtual_goal = []; agents(i).sensed_obstacles = [];
    agents(i).saw_obstacle_last_frame = false; agents(i).last_obstacle_point = [];
    agents(i).last_obstacle_angle = 0; agents(i).watchdog_timer = 0;
    agents(i).stagnation_pos = agents(i).pos; agents(i).last_obstacle_point_history = [];
end

% (Scenario setup remains the same)
switch scenario
    case 'zigzag', params.goal = [150; 0]; obstacles = {define_zigzag_obstacle()};
    case 'two_circles', params.goal = [150; 0];
        obs1 = define_circle_obstacle(75, 20, 15);
        obs2 = define_circle_obstacle(75, -20, 15);
        obstacles = {obs1, obs2};
    case 'semicircle', params.goal = [150; 0]; obstacles = {define_semicircle_obstacle(100, 0, 30)};
end
for i = 1:params.N, agents(i).virtual_goal = params.goal; end

num_steps = ceil(params.t_max / params.dt);
pos_history = zeros(2, params.N, num_steps);

%% Create and Size the Figure Window ONCE
figure;
set(gcf, 'Position', [200, 200, 900, 900]);

%% Main Simulation Loop
disp(['Starting final simulation for scenario: ' scenario]);
for k = 1:num_steps
    t = k * params.dt;
    updated_agents = agents; 
    
    for i = 1:params.N
        current_agent = agents(i);
        neighbors_idx = find_neighbors(i, agents, params.rc);
        updated_agents(i).sensed_obstacles = detect_obstacles(current_agent, obstacles, params.rs);
        
        % FIX: Pass 'params' to the update_agent_status function
        updated_agents(i) = update_agent_status(updated_agents(i), params);
        
        [updated_agents(i).virtual_goal, updated_agents(i).status] = calculate_virtual_goal(updated_agents(i), params);
        updated_agents(i).saw_obstacle_last_frame = ~isempty(current_agent.sensed_obstacles);
    end
    
    % (The rest of the script is correct and remains the same)
    final_agents = updated_agents;
    for i = 1:params.N
        neighbors_idx = find_neighbors(i, agents, params.rc);
        u_total = calculate_control_inputs(updated_agents(i), agents(neighbors_idx), params);
        final_agents(i).vel = updated_agents(i).vel + u_total * params.dt;
        if norm(final_agents(i).vel) > params.v_max, final_agents(i).vel = (final_agents(i).vel / norm(final_agents(i).vel)) * params.v_max; end
        final_agents(i).pos = updated_agents(i).pos + final_agents(i).vel * params.dt;
        if norm(final_agents(i).pos - final_agents(i).stagnation_pos) > 1.5, final_agents(i).watchdog_timer = 0; final_agents(i).stagnation_pos = final_agents(i).pos;
        else, final_agents(i).watchdog_timer = final_agents(i).watchdog_timer + params.dt; end
        if final_agents(i).watchdog_timer > 15, final_agents(i).status = 0; final_agents(i).watchdog_timer = 0; end
    end
    agents = final_agents;
    pos_history(:,:,k) = [agents.pos];
    if mod(k, 50) == 0, plot_simulation_state(agents, obstacles, params.goal, t, params.rc); drawnow; end
end
disp('Simulation finished.');

% (Final plotting section remains the same)

figure; hold on; title('Agent Trajectories');
for i=1:length(obstacles), plot(obstacles{i}(1,:), obstacles{i}(2,:), 'k-', 'LineWidth', 3); end
plot(params.goal(1), params.goal(2), 'r*', 'MarkerSize', 12, 'LineWidth', 2);
for i = 1:params.N, plot(squeeze(pos_history(1,i,:)), squeeze(pos_history(2,i,:))); end
xlabel('X-Position'); ylabel('Y-Position'); axis equal; grid on;