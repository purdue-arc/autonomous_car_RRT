function [state_tree, parents] = extend(state_tree, rand_pos, parents)
    num_nodes = size(state_tree, 1);
    
    %%% Constants
    
    % Simulation
    dt = 0.1;
    
    % Control
    steering_angle_range = pi/2;
    max_velocity = 3;
    velocity_std_dev = 1;
    
    % Physical
    length = 0.4;
    width = 0.2;
    mass = 4;
    inertia = 1;
    
    cf = 0.1; % Front cornering stiffness coeff
    cr = 0.1; % Rear cornering stiffness coeff
    lf = length/2; % Distance from center of gravity to front wheel
    lr = length/2; % Distance from center of gravity to rear wheel

    rand_pos_x = rand_pos(1);
    rand_pos_y = rand_pos(2);
    
    % This runs through whole tree and finds nearest point
    min_dist = inf;
    min_index = -1;
    for i = 1:num_nodes
        curr_state = state_tree(i, :);
        curr_pos_x = curr_state(1);
        curr_pos_y = curr_state(2);
        % Archimedes distance
        curr_dist = sqrt((rand_pos_x - curr_pos_x)^2 + (rand_pos_y - curr_pos_y)^2);
        if curr_dist < min_dist
            min_dist = curr_dist;
            min_index = i;
        end
    end
    
    % From the selected node, come up with a random control vector (could
    % experiment with weighting toward rand_pos in the future)
    rand_steering_angle = rand(1) * steering_angle_range - steering_angle_range/2;
    %rand_velocity = rand(1) * max_velocity; % Maybe use Gaussian in future to weight for low speeds?
    rand_velocity = abs(normrnd(0, velocity_std_dev));
    
        
    % Come up with new state
    new_state = integrater(state_tree(min_index, :), dt, length, width, mass, rand_velocity, cf, cr, lf, lr, inertia, rand_steering_angle);

    % Add new state to tree
    num_nodes = num_nodes + 1;
    state_tree(num_nodes,:) = new_state;
    parents(num_nodes) = min_index;
        
    % Plot for debugging
    plot(new_state(1), new_state(2), '*');
    line([new_state(1), state_tree(min_index,1)], [new_state(2), state_tree(min_index,2)]);
end
