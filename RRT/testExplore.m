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
vector_count = 45;          % Number of vectors to cast, increases accuracy, but also calculation time
view_width = deg2rad(90);   % Field of view of the robot
max_distance = 10;          % Max distance to consider viewable by robot (linear falloff)
observation_cutoff = 0.0;   % What is an acceptable difference from 1 or 0 in order to round to one or the other
          
map = ExploratoryMap(x_min, x_max, y_min, y_max, scale, simple_map, vector_count, view_width, max_distance, observation_cutoff);

state = [0.5, 0.5, pi/4, 0, 0]; % [x CG, y CG, theta, lateral speed(vy), yaw rate(r or thetadot)]

state_tree(1,:) = state;
parents = 0;
control_tree = [0, 0];

knowledge = map.evaluate_state(state)

view = map.execute_state(state);


% Display the map
    % Figure Position
    %set(gcf, 'Position', [0 0 1280 720]);

colormap(flipud(gray));
subplot(1,2,1);
axis([x_min x_max y_min y_max], 'square');
imagesc('XData',[x_min+1/(scale*2) x_max-1/(scale*2)],'YData',[y_max-1/(scale*2) y_min+1/(scale*2)],'CData',map.obstacle_array);
hold on;
scatter(view(:,1), view(:,2), round(view(:,3)*24)+1);
scatter(state(1), state(2), 'filled');
subplot(1,2,2);
axis([x_min x_max y_min y_max], 'square');
imagesc('XData',[x_min+1/(scale*2) x_max-1/(scale*2)],'YData',[y_max-1/(scale*2) y_min+1/(scale*2)],'CData',map.observation_array);
hold on;
scatter(state(1), state(2), 'filled');

state_tree(1,:) = state;
parents = 0;
control_tree = [0, 0];

for i = 2:500
    % Pass this to extend function and add the resulting state to the array
    [state_tree, parents, control_tree] = extend(state_tree, parents, control_tree, map);
end

scatter(state_tree(:,1), state_tree(:,2), '*');

% Find path
%goal = [rand(1) * (x_max - x_min - 2*radius) + x_min + radius, rand(1) * (y_max - y_min - 2*radius) + y_min + radius];
% goal = [8.3785, 5.7273];
% [path, length] = evaluateTree(state_tree, parents, goal, radius);
