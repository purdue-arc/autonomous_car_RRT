close all
clear variables

% Arbitrary values for the test
x_min = 0;
x_max = 10;
y_min = 0;
y_max = 10;
radius = 0.25;

% Plot / video export values
plot_result = true;
create_video = true;

state = [0.5, 0.5, pi/4, 0, 0]; % [x CG, y CG, theta, lateral speed(vy), yaw rate(r or thetadot)]

state_tree(1,:) = state;
parents = 0;
control_tree = [0, 0];

% Function handle for creating random points
rand_pos_funct = @() [rand(1) * (x_max - x_min) + x_min, rand(1) * (y_max - y_min) + y_min];

for i = 2:50
    % Pass this to extend function and add the resulting state to the array
    [state_tree, parents, control_tree] = extend(state_tree, parents, control_tree, rand_pos_funct, [x_min, x_max, y_min, y_max]);
end

% Find path
goal = [rand(1) * (x_max - x_min) + x_min, rand(1) * (y_max - y_min) + y_min];
[path, length] = evaluateTree(state_tree, parents, goal, radius);

% Plot for debugging / video export
if plot_result
    figure('Position', [0 0 1920 1080]);
    hold on
    axis([0 10 0 10]);
    if create_video
        vid = VideoWriter('RRT_example');
        vid.FrameRate = 10;
        open(vid);
    end

    for i = 1:size(state_tree, 1)
        curr_state = state_tree(i,:);
        plot(curr_state(1), curr_state(2), '*');
        % If it has a parent, plot a line
        if parents(i) ~= 0
            curr_parent = state_tree(parents(i),:);
            line([curr_state(1), curr_parent(1)], [curr_state(2), curr_parent(2)], 'Color', 'blue', 'LineStyle',':');
        end
        if create_video
            frame = getframe(gcf);
            writeVideo(vid, frame);
        end
    end
    viscircles(goal, radius);

    states = state_tree(path(1:length), 1:2);
    line(states(:, 1), states(:, 2), 'Color', 'red');

    for i = 1:length
        curr_state = state_tree(path(i), 1:2);
        plot(curr_state(1), curr_state(2), 'hr');
    end
    if create_video
        close(vid);
    end
end