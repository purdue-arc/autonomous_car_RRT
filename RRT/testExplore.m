close all
clear variables

% Arbitrary values for the test
x_min = 0;
x_max = 50;
y_min = 0;
y_max = 50;
num_steps = 120;    % When to stop exploring (in future use while loop?)

create_video = true;

% Values for exploration
% simple_map = [0 0 0 0 1;
%               0 0 1 0 0;
%               0 1 1 1 0;
%               0 0 0 1 0;
%               0 0 0 0 0];

% simple_map =  [ 1 0 0 0 0 0 1 1 1 1;
%                 1 0 0 0 0 0 1 0 1 1;
%                 1 0 0 1 0 0 1 0 0 1;
%                 0 0 0 1 0 0 0 0 0 1;
%                 0 0 1 1 1 0 0 0 0 0;
%                 0 0 0 0 1 0 0 0 0 0;
%                 0 0 0 0 0 0 0 1 0 0;
%                 1 0 0 0 0 0 0 1 0 0;
%                 0 0 0 0 0 0 0 0 0 1;
%                 0 0 0 1 0 0 0 0 0 1];

filename = "100_map.mat";
mat = matfile(filename);
simple_map = mat.obstacle_matrix_mod;

scale = 10;                 % there should be how many cell-lengths per unit (meter)
execution_vector_count = 90;% Number of vectors to cast when executing a postion, increases accuracy, but also calculation time
evaluation_vector_count = 5;% Number of vectors to cast when evaluation a position. higher increases accuracy, but also evaluation time.
view_width = deg2rad(90);   % Field of view of the robot
max_distance = 10;          % Max distance to consider viewable by robot (linear falloff)
obstacle_cutoff = 0.55;     % At what point do you assume something is an obstacle
num_nodes = 250;            % How many nodes to generate per step
          
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
set(gcf, 'Position', [300 200 1280 720]);
colormap(flipud(gray));

if create_video
    set(gcf,'menubar','none')
    vid = VideoWriter('Exploration_example', 'MPEG-4');
    vid.FrameRate = 6;
    open(vid);
end

% Perform exploration
for i = 2:num_steps+1
    % Perform last movement
    cur_state = state_tree(i-1,:);
    cur_view =  map.execute_state(cur_state);
    
    % Update the graphs
    clf;                                                        % Clear old stuff (since it can hang slightly off screen)
    ax = subplot(1,2,1);                                        % Left plot
    title("Obstacle Map");
    xlabel("X Position (m)");
    ylabel("Y Position (m)");
    hold on;
    axis([x_min x_max y_min y_max], 'square');                  % Set axis
                                                                % Plot image
    imagesc('XData', [x_min + 0.5/map.scale,  x_max - 0.5/map.scale], 'YData', [x_max - 0.5/map.scale,  x_min + 0.5/map.scale], 'CData', map.obstacle_array);
    ax.ColorOrderIndex = 1;                                     % Make vis blue for consistency
    scatter(cur_view(:,1), cur_view(:,2), round(cur_view(:,3)*9)+1);       % Visibility
    scatter(cur_state(1), cur_state(2), 'filled');              % Car
    
    ax = subplot(1,2,2);                                        % Right plot
    title("Observation Map");
    xlabel("X Position (m)");
    ylabel("Y Position (m)");
    hold on;
    axis([x_min x_max y_min y_max], 'square');                  % Set axis
                                                                % Plot image
    imagesc('XData', [x_min + 0.5/map.scale,  x_max - 0.5/map.scale], 'YData', [x_max - 0.5/map.scale,  x_min + 0.5/map.scale], 'CData', map.observation_array);
    plot(state_tree(1:i-1,1), state_tree(1:i-1,2), 'r*:');      % Plot the path taken
    
    if i <= num_steps
        % Choose next path
        [next_state, next_control, next_value, rrt_tree, rrt_parents] = explore(map, cur_state, num_nodes);

        % Update arrays
        state_tree(i,:) = next_state;
        control_tree(i,:) = next_control;
        value_tree(i) = next_value;
    
        % Plot the RRT tree for debug
        ax.ColorOrderIndex = 4;                                     % Get some nice purple
        point_array = plot(rrt_tree(:,1), rrt_tree(:,2), '*');      % Plot the nodes
        % Plot the lines
        x_points = [rrt_tree(2:end, 1), rrt_tree(rrt_parents(2:end), 1)]';
        y_points = [rrt_tree(2:end, 2), rrt_tree(rrt_parents(2:end), 2)]';
        line_array = line(x_points, y_points, 'Color', 'blue', 'LineStyle', ':');
    end

    ax.ColorOrderIndex = 2;                                     % Get some nice orange
    scatter(cur_state(1), cur_state(2), 'filled');              % Car
    drawnow;
    if create_video
        frame = getframe(gcf);
        writeVideo(vid, frame);
        if mod(i,10) == 0
            fprintf('progress: %d / %d\n', i, num_steps);
        end
    end
end

if create_video
    close(vid);
end
