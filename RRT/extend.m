function [state_tree, parents, control_tree] = extend(state_tree, parents, control_tree, map)
    num_nodes = size(state_tree, 1);
    
    %%% Constants
    
    % Simulation
    dt = 0.1;
    
    % Control
    steering_angle_range = pi/2;
    max_velocity = 5;
    min_velocity = 1.5;
%     velocity_std_dev = 1;
    length = 0.4;
    
    for attempt = 1:100 % Up to 100 tries before it aborts
        [rand_pos_x, rand_pos_y] = map.gen_rand_pos();

        % This runs through whole tree and finds nearest point
        min_dist = inf;
        min_index = -1;
        for node = 1:num_nodes
            curr_state = state_tree(node, :);
            curr_pos_x = curr_state(1);
            curr_pos_y = curr_state(2);
            % Archimedes distance
            curr_dist = sqrt((rand_pos_x - curr_pos_x)^2 + (rand_pos_y - curr_pos_y)^2);
            if curr_dist < min_dist
                min_dist = curr_dist;
                min_index = node;
            end
        end

        % From the selected node, come up with a random control vector (could experiment with weighting toward rand_pos in the future)
        rand_steering_angle = rand(1) * steering_angle_range - steering_angle_range/2;
        rand_velocity = rand(1) * (max_velocity - min_velocity) + min_velocity; % Maybe use Gaussian in future to weight for low speeds?
        %rand_velocity = abs(normrnd(0, velocity_std_dev));


        % Come up with new state
        new_state = integraterKinematic(state_tree(min_index, :), dt, length, rand_velocity, rand_steering_angle);
        
        if map.check_pos(new_state(1), new_state(2))
            % Within bounds, continue
            % Otherwise, try again
            break;
        end
        if attempt == 100
            % Failed
            fprintf('Failed to generate state in bounds for point at index %d\n', num_nodes+1);
        end
    end
    
    % Add new state to tree
    num_nodes = num_nodes + 1;
    state_tree(num_nodes,:) = new_state;
    parents(num_nodes) = min_index;
    control_tree(num_nodes, :) = [rand_steering_angle, rand_velocity];
end
