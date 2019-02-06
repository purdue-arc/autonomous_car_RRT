function [next_state, next_control, next_value, state_tree, parents] = explore(map, cur_state, num_nodes)

    % Create RRT arrays
    state_tree = zeros(num_nodes, 5);   % State at each node
    parents = zeros(num_nodes, 1);      % Parent of each node (index into state_tree)
    control_tree = zeros(num_nodes, 2); % Control to get to each node from parent
    
    % Populate RRT arrays with initial data
    state_tree(1,:) = cur_state;

    % Perform RRT
    for i = 2:num_nodes
        % Pass this to extend function and add the resulting state to the array
        [state_tree, parents, control_tree] = extend(state_tree, parents, control_tree, map, i, @(~)map.gen_rand_explore_pos(cur_state(1), cur_state(2)), @map.check_pos_explore);
    end

    % Create knowledge and cost array
    knowledge_tree = zeros(num_nodes, 2); % Col 1: self knowledge, col 2: knowledge of self + children

    % Compute knowledge and cost
    for i = num_nodes:-1:2 % Work backwards through tree
        parent_index = parents(i);
        % Calc knowledge and distance values
        knowledge = map.evaluate_state(state_tree(i,:));
        
        % Update self
        knowledge_tree(i,:) = knowledge_tree(i,:) + [knowledge, knowledge];

        % Update parent
        knowledge_tree(parent_index,2) = knowledge_tree(parent_index,2) + knowledge_tree(i,2);  % Add self + children knowledge to parent
    end
    
    % Determine the indices of the possible next states
    possible_states = find(parents == 1);
    
    % Determine value of each node
    possible_values = knowledge_tree(possible_states,2);

    % Determine the value and index of the next state with the max value
    [next_value, next_state_index] = max(possible_values);

    % Determine the state and required control
    next_state = state_tree(possible_states(next_state_index), :);
    next_control = control_tree(possible_states(next_state_index), :);
