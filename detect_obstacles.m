function all_points = detect_obstacles(agent, obstacles, sensor_range)
    % FINAL CORRECTED VERSION
    
    all_points = [];
    
    for obs_idx = 1:length(obstacles)
        verts = obstacles{obs_idx};
        
        % FIX: Renamed loop variable from 'i' to 'seg_idx' to prevent a scoping conflict
        % with the main simulation loop's agent iterator 'i'.
        for seg_idx = 1:(size(verts, 2) - 1)
            p1 = verts(:, seg_idx);
            p2 = verts(:, seg_idx+1);
            
            v = p2 - p1;
            w = agent.pos - p1;
            
            if norm(v) < 1e-9, t=0; else, t = max(0, min(1, (w' * v) / (v' * v))); end
            closest_pt = p1 + t * v;
            
            if norm(agent.pos - closest_pt) < sensor_range
                all_points = [all_points, closest_pt];
            end
        end
    end
end