close all
clear variables

% Arbitrary values for the test
x_min = 0;
x_max = 10;
y_min = 0;
y_max = 10;
num_steps = 120;    % When to stop exploring (in future use while loop?)

create_video = true;

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

cur_state = [0.5, 0.5, pi/4, 0, 0]; % [x CG, y CG, theta, lateral speed(vy), yaw rate(r or thetadot)]

% Create exploration arrays
state_tree = zeros(num_steps, 5);   % State at each node
control_tree = zeros(num_steps, 2); % Control to get to each node from parent
value_tree = zeros(num_steps, 1);   % The value of each move (prediction)
% Note that no parent array is needed since each node is child of previous

% Populate exploration arrays
state_tree(1,:) = cur_state;
cur_view = map.execute_state(cur_state);

% Initial plot
set(gcf, 'Position', [0 0 1280 720]);
clf;                                                        % Clear old stuff (since it can hang slightly off screen)
colormap(flipud(gray));
ax = subplot(1,2,1);                                        % Left plot
hold on;
axis([x_min x_max y_min y_max], 'square');                  % Set axis
                                                            % Plot image
imagesc('XData',[x_min+1/(scale*2) x_max-1/(scale*2)],'YData',[y_max-1/(scale*2) y_min+1/(scale*2)],'CData',map.obstacle_array);
ax.ColorOrderIndex = 1;                                     % Make vis blue for consistency
scatter(cur_view(:,1), cur_view(:,2), round(cur_view(:,3)*9)+1);       % Visibility
scatter(cur_state(1), cur_state(2), 'filled');              % Car

ax = subplot(1,2,2);                                        % Right plot
hold on;
axis([x_min x_max y_min y_max], 'square');                  % Set axis
                                                            % Plot image
imagesc('XData',[x_min+1/(scale*2) x_max-1/(scale*2)],'YData',[y_max-1/(scale*2) y_min+1/(scale*2)],'CData',map.observation_array);
ax.ColorOrderIndex = 2;                                     % Get some nice orange
scatter(cur_state(1), cur_state(2), 100, 'filled');       % Car

drawnow;

if create_video
    set(gcf,'menubar','none')
    vid = VideoWriter('Exploration_example', 'MPEG-4');
    vid.FrameRate = 6;
    open(vid);
    frame = getframe(gcf);
    writeVideo(vid, frame);
end

% Perform exploration
for i = 2:num_steps
    % Choose next path
    cur_state = state_tree(i-1,:);
    [next_state, next_control, next_value, rrt_tree] = explore(map, cur_state, num_nodes);
    
    % Update arrays
    state_tree(i,:) = next_state;
    control_tree(i,:) = next_control;
    value_tree(i) = next_value;
    
    % Perform the movement
    cur_view =  map.execute_state(next_state);
    
    % Update the graphs
%     set(gcf, 'Position', [0 0 1280 720]);
    clf;                                                        % Clear old stuff (since it can hang slightly off screen)
%     colormap(flipud(gray));
    ax = subplot(1,2,1);                                        % Left plot
    hold on;
    axis([x_min x_max y_min y_max], 'square');                  % Set axis
                                                                % Plot image
    imagesc('XData',[x_min+1/(scale*2) x_max-1/(scale*2)],'YData',[y_max-1/(scale*2) y_min+1/(scale*2)],'CData',map.obstacle_array);
    ax.ColorOrderIndex = 1;                                     % Make vis blue for consistency
    scatter(cur_view(:,1), cur_view(:,2), round(cur_view(:,3)*9)+1);       % Visibility
    scatter(next_state(1), next_state(2), 'filled');              % Car

    ax = subplot(1,2,2);                                        % Right plot
    hold on;
    axis([x_min x_max y_min y_max], 'square');                  % Set axis
                                                                % Plot image
    imagesc('XData',[x_min+1/(scale*2) x_max-1/(scale*2)],'YData',[y_max-1/(scale*2) y_min+1/(scale*2)],'CData',map.observation_array);
    plot(state_tree(1:i,1), state_tree(1:i,2), 'b*:');          % Plot the path taken
    ax.ColorOrderIndex = 2;                                     % Get some nice orange
    scatter(next_state(1), next_state(2), 100, 'filled');       % Car
    
    drawnow;
    
    if create_video
        frame = getframe(gcf);
        writeVideo(vid, frame);
        fprintf('wrote');
        if mod(i,10) == 0
            fprintf('progress: %d / %d\n', i, num_steps);
        end
    end
end

if create_video
    close(vid);
end

% Display the map
    % Figure Position
    %set(gcf, 'Position', [0 0 1280 720]);

% colormap(flipud(gray));
% subplot(1,2,1);                                             % Left plot
% hold on;
% axis([x_min x_max y_min y_max], 'square');                  % Set axis
%                                                             % Plot image
% imagesc('XData',[x_min+1/(scale*2) x_max-1/(scale*2)],'YData',[y_max-1/(scale*2) y_min+1/(scale*2)],'CData',map.obstacle_array);
% scatter(cur_view(:,1), cur_view(:,2), round(cur_view(:,3)*24)+1);       % Visibility
% scatter(cur_state(1), cur_state(2), 'filled');              % Car
% 
% ax = subplot(1,2,2);                                        % Right plot
% hold on;
% axis([x_min x_max y_min y_max], 'square');                  % Set axis
%                                                             % Plot image
% imagesc('XData',[x_min+1/(scale*2) x_max-1/(scale*2)],'YData',[y_max-1/(scale*2) y_min+1/(scale*2)],'CData',map.observation_array);
% ax.ColorOrderIndex = 2;                                     % Get some nice orange
% scatter(cur_state(1), cur_state(2), 100, 'filled');         % Car
% ax.ColorOrderIndex = 4;                                     % Get some nice purple
% point_array = plot(state_tree(:,1), state_tree(:,2), '*');  % Plot the nodes
% scatter(next_state(1), next_state(2), 'filled');            % Next State
% 
% % lets make some lines
% x_points = [state_tree(2:end, 1), state_tree(parents(2:end), 1)]';
% y_points = [state_tree(2:end, 2), state_tree(parents(2:end), 2)]';
% line_array = line(x_points, y_points, 'Color', 'blue', 'LineStyle', ':');

% Find path
%goal = [rand(1) * (x_max - x_min - 2*radius) + x_min + radius, rand(1) * (y_max - y_min - 2*radius) + y_min + radius];
% goal = [8.3785, 5.7273];
% [path, length] = evaluateTree(state_tree, parents, goal, radius);
