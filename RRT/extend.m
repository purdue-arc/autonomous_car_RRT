function [state_tree, parents, control_tree] = extend(state_tree, parents, control_tree, map, cur_node, generator, checker)
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
        [rand_pos_x, rand_pos_y] = generator();
        
        % Find nearest point
        dist = sqrt((state_tree(1:(cur_node-1),1) - rand_pos_x).^2 + (state_tree(1:(cur_node-1),2) - rand_pos_y).^2);
        [~, min_index] = min(dist);

        % From the selected node, come up with a random control vector
        rand_steering_angle = rand * steering_angle_range - steering_angle_range/2;
        rand_velocity = rand * (max_velocity - min_velocity) + min_velocity;

        % Come up with new state
        new_state = integraterKinematic(state_tree(min_index, :), dt, length, rand_velocity, rand_steering_angle);
        
        if checker(new_state(1), new_state(2)) %(execution && map.check_pos(new_state(1), new_state(2))) || (~execution && map.check_pos_explore(new_state(1), new_state(2)))
            success = true;
            % Within bounds and not on an obstacle point
            % Need to check every point along the line too
            parent_x = state_tree(min_index,1) * map.scale;
            parent_y = state_tree(min_index,2) * map.scale;
            dist = sqrt((parent_x - new_state(1)*map.scale)^2 + (parent_y - new_state(2)*map.scale)^2) * map.scale;
            for d=1:dist
                % Internal position of end of line going from parent to child
                x = round(parent_x + d * cos(new_state(3)));
                y = round(parent_y + d * sin(new_state(3)));
                if ~checker(x/map.scale, y/map.scale) %execution && ~map.check_pos(x/map.scale, y/map.scale) || (~execution && ~map.check_pos_explore(x/map.scale, y/map.scale))
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
            new_state = state_tree(min_index, :);
            rand_steering_angle = 0;
            rand_velocity = 0;
        end
    end
    
    % Add new state to tree
    state_tree(cur_node,:) = new_state;
    parents(cur_node) = min_index;
    control_tree(cur_node,:) = [rand_steering_angle, rand_velocity];
end
