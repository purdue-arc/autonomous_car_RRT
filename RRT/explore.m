function [next_state, next_control, next_value, state_tree] = explore(map, cur_state, num_nodes)

    % Create RRT arrays
    state_tree = zeros(num_nodes, 5);   % State at each node
    parents = zeros(num_nodes, 1);      % Parent of each node (index into state_tree)
    control_tree = zeros(num_nodes, 2); % Control to get to each node from parent
    
    % Populate RRT arrays with initial data
    state_tree(1,:) = cur_state;

    % Perform RRT
    for i = 2:num_nodes
        % Pass this to extend function and add the resulting state to the array
        [state_tree, parents, control_tree] = extend(state_tree, parents, control_tree, map, i);
    end

    % Create knowledge and cost array
    utility_tree = zeros(num_nodes, 2); % Col 1: self knowledge, col 2: knowledge of self + children
    cost_tree = zeros(num_nodes, 2);    % Col 1:  Number of children, col 2: Cost of children (number of total steps)

    % Compute knowledge and cost
    for i = num_nodes:-1:2 % Work backwards through tree
        % Update self
        knowledge = map.evaluate_state(state_tree(i,:));
        utility_tree(i,:) = utility_tree(i,:) + [knowledge, knowledge];

        % Update parent
        num_children = cost_tree(i,1) + 1;  % children of parent = self + 1
        parent_index = parents(i);
        utility_tree(parent_index,2) = utility_tree(parent_index,2) + utility_tree(i,2);  % Add self + children knowledge to parent
        cost_tree(parent_index,:) = cost_tree(parent_index,:) + [num_children, cost_tree(i,2) + num_children];  % Add children and children cost
    end

    % Determine mean knowledge per step
    avg_knowledge_per_step = utility_tree(1,2) / cost_tree(1,2);

    % Determine value of each node
    value_tree = utility_tree(:,2) - avg_knowledge_per_step * cost_tree(:,2);

    % Determine the indices of the possible next states
    possible_states = find(parents == 1);

    % Determine the value and index of the next state with the max value
    [next_value, next_state_index] = max(value_tree(possible_states));

    % Determine the state and required control
    next_state = state_tree(possible_states(next_state_index), :);
    next_control = control_tree(possible_states(next_state_index), :);

% colormap(flipud(gray));
% subplot(1,2,1);                                             % Left plot
% hold on;
% axis([x_min x_max y_min y_max], 'square');                  % Set axis
%                                                             % Plot image
% imagesc('XData',[x_min+1/(scale*2) x_max-1/(scale*2)],'YData',[y_max-1/(scale*2) y_min+1/(scale*2)],'CData',map.obstacle_array);
% scatter(view(:,1), view(:,2), round(view(:,3)*24)+1);       % Visibility
% scatter(state(1), state(2), 'filled');                      % Car
% 
% ax = subplot(1,2,2);                                        % Right plot
% hold on;
% axis([x_min x_max y_min y_max], 'square');                  % Set axis
%                                                             % Plot image
% imagesc('XData',[x_min+1/(scale*2) x_max-1/(scale*2)],'YData',[y_max-1/(scale*2) y_min+1/(scale*2)],'CData',map.observation_array);
% ax.ColorOrderIndex = 2;                                     % Get some nice orange
% scatter(state(1), state(2), 100, 'filled');                 % Car
% ax.ColorOrderIndex = 4;                                     % Get some nice purple
% point_array = plot(state_tree(:,1), state_tree(:,2), '*');  % Plot the nodes
% scatter(next_state(1), next_state(2), 'filled');                      % Next State
% 
% % lets make some lines
% x_points = [state_tree(2:end, 1), state_tree(parents(2:end), 1)]';
% y_points = [state_tree(2:end, 2), state_tree(parents(2:end), 2)]';
% line_array = line(x_points, y_points, 'Color', 'blue', 'LineStyle', ':');
