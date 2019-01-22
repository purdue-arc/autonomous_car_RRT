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

num_nodes_per_step = 500;   % How many nodes to generate per step
          
map = ExploratoryMap(x_min, x_max, y_min, y_max, scale, simple_map, evaluation_vector_count, execution_vector_count, view_width, max_distance, obstacle_cutoff);

state = [0.5, 0.5, pi/4, 0, 0]; % [x CG, y CG, theta, lateral speed(vy), yaw rate(r or thetadot)]

knowledge = map.evaluate_state(state);
view = map.execute_state(state);

state_tree(1,:) = state;
parents = 0;
control_tree = [0, 0];

for i = 2:num_nodes_per_step
    % Pass this to extend function and add the resulting state to the array
    [state_tree, parents, control_tree] = extend(state_tree, parents, control_tree, map);
end

knowledgeArray = zeros(num_nodes_per_step, 1);
for i = 2:num_nodes_per_step
    knowledgeArray(i) = map.evaluate_state(state_tree(i,:));
end


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
point_array = plot(state_tree(:,1), state_tree(:,2), '*');   % Plot the nodes

% lets make some lines
x_points = [state_tree(2:end, 1), state_tree(parents(2:end), 1)]';
y_points = [state_tree(2:end, 2), state_tree(parents(2:end), 2)]';
line_array = line(x_points, y_points, 'Color', 'blue', 'LineStyle', ':');

% Find path
%goal = [rand(1) * (x_max - x_min - 2*radius) + x_min + radius, rand(1) * (y_max - y_min - 2*radius) + y_min + radius];
% goal = [8.3785, 5.7273];
% [path, length] = evaluateTree(state_tree, parents, goal, radius);
