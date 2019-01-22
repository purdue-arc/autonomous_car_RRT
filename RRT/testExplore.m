close all
clear variables

% Arbitrary values for the test
x_min = 0;
x_max = 10;
y_min = 0;
y_max = 10;

% Values for exploration
simple_map = [0 0 0 0 1;
              0 0 1 0 0;
              0 1 1 1 0;
              0 0 0 1 0;
              0 0 0 0 0];
scale = 10;                 % there should be how many cell-lengths per unit (meter)
execution_vector_count = 45;% Number of vectors to cast when executing a postion, increases accuracy, but also calculation time
evaluation_vector_count = 5;% Number of vectors to cast when evaluation a position. higher increases accuracy, but also evaluation time.
view_width = deg2rad(90);   % Field of view of the robot
max_distance = 10;          % Max distance to consider viewable by robot (linear falloff)
obstacle_cutoff = 0.75;     % At what point do you assume something is an obstacle

num_nodes = 250;   % How many nodes to generate per step
          
map = ExploratoryMap(x_min, x_max, y_min, y_max, scale, simple_map, evaluation_vector_count, execution_vector_count, view_width, max_distance, obstacle_cutoff);

state = [0.5, 0.5, pi/4, 0, 0]; % [x CG, y CG, theta, lateral speed(vy), yaw rate(r or thetadot)]

knowledge = map.evaluate_state(state);
view = map.execute_state(state);

state_tree(1,:) = state;
parents = 0;
control_tree = [0, 0];

% Perform RRT
for i = 2:num_nodes
    % Pass this to extend function and add the resulting state to the array
    [state_tree, parents, control_tree] = extend(state_tree, parents, control_tree, map);
end

% Compute estimated knowledge
knowledge_tree = zeros(num_nodes-1, 1);  % Knowledge estimate per node
for i = 2:num_nodes
    knowledge_tree(i) = map.evaluate_state(state_tree(i,1:3));
end

% Determine mean knowledge and compute cost per step


% Compute values for each node
cost_utility_tree = zeros(num_nodes, 4);  % Col 1: knowledge, col 2: children, col 3: knowledge sum, col 4: children sum
for i = num_nodes:-1:2 % Work backwards through tree
    % Update self
    knowledge = map.evaluate_state(state_tree(i,1:3));
    cost_utility_tree(i,:) = cost_utility_tree(i,:) + [knowledge, 0, knowledge, 0];
    
    % Update parent
    num_children = cost_utility_tree(i,2) + 1;                 % children of parent = self + 1
    parent_index = parents(i);
    cost_utility_tree(parent_index,:) = cost_utility_tree(parent_index,:) + [0, num_children, cost_utility_tree(i,3), num_children + cost_utility_tree(i,4)];
end

avg_cost_per_step = cost_utility_tree(1,3) / cost_utility_tree(1,4);

value_tree = cost_utility_tree(:,3) - avg_cost_per_step * cost_utility_tree(:,4);

possible_states = find(parents == 1);

[next_state_value, next_state_index] = max(value_tree(possible_states));

next_state = state_tree(possible_states(next_state_index), :);
next_control = control_tree(possible_states(next_state_index), :);

% Display the map
    % Figure Position
    %set(gcf, 'Position', [0 0 1280 720]);

colormap(flipud(gray));
subplot(1,2,1);                                             % Left plot
hold on;
axis([x_min x_max y_min y_max], 'square');                  % Set axis
                                                            % Plot image
imagesc('XData',[x_min+1/(scale*2) x_max-1/(scale*2)],'YData',[y_max-1/(scale*2) y_min+1/(scale*2)],'CData',map.obstacle_array);
scatter(view(:,1), view(:,2), round(view(:,3)*24)+1);       % Visibility
scatter(state(1), state(2), 'filled');                      % Car

ax = subplot(1,2,2);                                        % Right plot
hold on;
axis([x_min x_max y_min y_max], 'square');                  % Set axis
                                                            % Plot image
imagesc('XData',[x_min+1/(scale*2) x_max-1/(scale*2)],'YData',[y_max-1/(scale*2) y_min+1/(scale*2)],'CData',map.observation_array);
ax.ColorOrderIndex = 2;                                     % Get some nice orange
scatter(state(1), state(2), 100, 'filled');                 % Car
ax.ColorOrderIndex = 4;                                     % Get some nice purple
point_array = plot(state_tree(:,1), state_tree(:,2), '*');  % Plot the nodes
scatter(next_state(1), next_state(2), 'filled');                      % Next State

% lets make some lines
x_points = [state_tree(2:end, 1), state_tree(parents(2:end), 1)]';
y_points = [state_tree(2:end, 2), state_tree(parents(2:end), 2)]';
line_array = line(x_points, y_points, 'Color', 'blue', 'LineStyle', ':');

% Find path
%goal = [rand(1) * (x_max - x_min - 2*radius) + x_min + radius, rand(1) * (y_max - y_min - 2*radius) + y_min + radius];
% goal = [8.3785, 5.7273];
% [path, length] = evaluateTree(state_tree, parents, goal, radius);
