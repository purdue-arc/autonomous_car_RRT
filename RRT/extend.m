function [state_tree, parents, control_tree] = extend(state_tree, parents, control_tree, map, cur_node)
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
        
        if map.check_pos(new_state(1), new_state(2))
            % Within bounds, continue
            % Otherwise, try again
            break;
        end
        if attempt == num_attempts
            % Failed
            fprintf('Failed to generate state in bounds for point at index %d\n', cur_node);
        end
    end
    
    % Add new state to tree
    state_tree(cur_node,:) = new_state;
    parents(cur_node) = nearest_node_index;
    control_tree(cur_node,:) = [rand_steering_angle, rand_velocity];
end
