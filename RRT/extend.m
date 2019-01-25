function [state_tree, parents, control_tree] = extend(state_tree, parents, control_tree, map, cur_node, execution)
    %%% Constants
    % Simulation
    dt = 0.1;
    
    % Arbitrary
    num_attempts = 100; % How many times to try to create new node until failure
    
    % Control
    steering_angle_range = pi/2;
    max_velocity = 5;
    min_velocity = 1.5;
    length = 0.4;
    
    for attempt = 1:num_attempts % Up to num_attempts tries before it aborts
        % Generate random pos
        [rand_pos_x, rand_pos_y] = map.gen_rand_pos();

        % Find nearest point
        min_dist = inf;
        nearest_node_index = -1;
        for node = 1:cur_node-1   % Only search tree up to where it is populated
            curr_state = state_tree(node, :);
            curr_pos_x = curr_state(1);
            curr_pos_y = curr_state(2);
            % Archimedes distance
            curr_dist = sqrt((rand_pos_x - curr_pos_x)^2 + (rand_pos_y - curr_pos_y)^2);
            if curr_dist < min_dist
                min_dist = curr_dist;
                nearest_node_index = node;
            end
        end

        % From the selected node, come up with a random control vector
        rand_steering_angle = rand * steering_angle_range - steering_angle_range/2;
        rand_velocity = rand * (max_velocity - min_velocity) + min_velocity;

        % Come up with new state
        new_state = integraterKinematic(state_tree(nearest_node_index, :), dt, length, rand_velocity, rand_steering_angle);
        
        if (execution && map.check_pos(new_state(1), new_state(2))) || (~execution && map.check_pos_explore(new_state(1), new_state(2)))
            success = true;
            % Within bounds and not on an obstacle point
            % Need to check every point along the line too
            parent_x = state_tree(nearest_node_index,1);
            parent_y = state_tree(nearest_node_index,2);
            dist = sqrt((parent_x - new_state(1))^2 + (parent_y - new_state(2))^2);
            for d=1:dist*map.scale
                % Internal position of end of line going from parent to child
                x = round(parent_x + d * cos(new_state(3)));
                y = round(parent_y + d * sin(new_state(3)));
                if execution && ~map.check_pos(x/map.scale, y/map.scale) || (~execution && ~map.check_pos_explore(x/map.scale, y/map.scale))
                    % Found an obstacle
                    % Set flag
                    success = false;
                    % No need to continue
                    break;
                end
            end
            if success
                % No obstacles found, break out of the loop and use this control and state
                break;
            end
            % Otherwise keep going and try new controls
        end
        if attempt == num_attempts
            % Failed
            fprintf('Failed to generate state in bounds for point at index %d, duplicating parent\n', cur_node);
            new_state = state_tree(nearest_node_index, :);
            rand_steering_angle = 0;
            rand_velocity = 0;
        end
    end
    
    % Add new state to tree
    state_tree(cur_node,:) = new_state;
    parents(cur_node) = nearest_node_index;
    control_tree(cur_node,:) = [rand_steering_angle, rand_velocity];
end
