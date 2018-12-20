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
          
map = ExploratoryMap(x_min, x_max, y_min, y_max, 10, simple_map);

% Display the map
colormap(flipud(gray));
image(true_map,'CDataMapping','scaled');

state = [0.5, 0.5, pi/4, 0, 0]; % [x CG, y CG, theta, lateral speed(vy), yaw rate(r or thetadot)]

state_tree(1,:) = state;
parents = 0;
control_tree = [0, 0];

% for i = 2:5000
%     % Create a new random position in the map
%     rand_pos = [rand(1) * (x_max - x_min) + x_min, rand(1) * (y_max - y_min) + y_min];
% 
%     % Pass this to extend function and add the resulting state to the array
%     [state_tree, parents, control_tree] = extend(state_tree, parents, control_tree, rand_pos);
% end
% 
% % Plot for debugging
% for i = 1:size(state_tree, 1)
%     curr_state = state_tree(i,:);
%     plot(curr_state(1), curr_state(2), '*');
%     % If it has a parent, plot a line
%     if parents(i) ~= 0
%         curr_parent = state_tree(parents(i),:);
%         line([curr_state(1), curr_parent(1)], [curr_state(2), curr_parent(2)], 'Color', 'blue', 'LineStyle',':');
%     end
% end
% 
% goal = [rand(1) * (x_max - x_min) + x_min, rand(1) * (y_max - y_min) + y_min];
% radius = 0.25;
% 
% viscircles(goal, radius);
% [path, length] = evaluateTree(state_tree, parents, goal, radius);
% 
% % Plot for debugging
% states = state_tree(path(1:length), 1:2);
% line(states(:, 1), states(:, 2), 'Color', 'red');
% 
% for i = 1:length
%     curr_state = state_tree(path(i), 1:2);
%     plot(curr_state(1), curr_state(2), 'hr');
% end