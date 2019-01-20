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
vector_count = 25;           % Number of vectors to cast, increases accuracy, but also calculation time
view_width = deg2rad(90);   % Field of view of the robot
max_distance = 10;          % Max distance to consider viewable by robot (linear falloff)
observation_cutoff = 0.1;   % What is an acceptable difference from 1 or 0 in order to round to one or the other
          
map = ExploratoryMap(x_min, x_max, y_min, y_max, scale, simple_map, vector_count, view_width, max_distance, observation_cutoff);

state = [0.5, 0.5, pi/4, 0, 0]; % [x CG, y CG, theta, lateral speed(vy), yaw rate(r or thetadot)]

state_tree(1,:) = state;
parents = 0;
control_tree = [0, 0];

visible_points = map.execute_state(state);

% Display the map
    % Figure Position
    %set(gcf, 'Position', [0 0 1280 720]);

colormap(flipud(gray));
subplot(1,2,1);
axis([x_min x_max y_min y_max], 'square');
imagesc('XData',[x_min+1/(scale*2) x_max-1/(scale*2)],'YData',[y_max-1/(scale*2) y_min+1/(scale*2)],'CData',map.obstacle_array);
hold on;
scatter(visible_points(:,1)/map.scale, visible_points(:,2)/map.scale, round(visible_points(:,3)*24)+1);
scatter(state(1), state(2), 'filled');
subplot(1,2,2);
axis([x_min x_max y_min y_max], 'square');
imagesc('XData',[x_min+1/(scale*2) x_max-1/(scale*2)],'YData',[y_max-1/(scale*2) y_min+1/(scale*2)],'CData',map.observation_array);
hold on;
scatter(state(1), state(2), 'filled');

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