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
create_video = false;

state = [0.5, 0.5, pi/4, 0, 0]; % [x CG, y CG, theta, lateral speed(vy), yaw rate(r or thetadot)]

state_tree(1,:) = state;
parents = 0;
control_tree = [0, 0];

% Function handle for creating random points
rand_pos_funct = @() [rand(1) * (x_max - x_min) + x_min, rand(1) * (y_max - y_min) + y_min];

for i = 2:5000
    % Pass this to extend function and add the resulting state to the array
    [state_tree, parents, control_tree] = extend(state_tree, parents, control_tree, rand_pos_funct, [x_min, x_max, y_min, y_max]);
end

% Find path
goal = [rand(1) * (x_max - x_min - 2*radius) + x_min + radius, rand(1) * (y_max - y_min - 2*radius) + y_min + radius];
[path, length] = evaluateTree(state_tree, parents, goal, radius);

% Plot for debugging / video export
if plot_result
    figure;
    hold on;
    axis([x_min x_max y_min y_max], 'square');
    if create_video
        set(gcf, 'Position', [0 0 1280 720]);
        set(gcf,'menubar','none')
        vid = VideoWriter('RRT_example', 'MPEG-4');
        vid.FrameRate = 60;
        open(vid);
    end
    
    plot_array = [;];
    for i = 1:size(state_tree, 1)
        curr_state = state_tree(i,:);
        new_point = plot(curr_state(1), curr_state(2), '*');
        plot_array(1,i) = new_point; % Add
        % If it has a parent, plot a line
        if parents(i) ~= 0
            curr_parent = state_tree(parents(i),:);
            new_line = line([curr_state(1), curr_parent(1)], [curr_state(2), curr_parent(2)], 'Color', 'blue', 'LineStyle',':');
            plot_array(2,i) = new_line;
        end
        if create_video
            frame = getframe(gcf);
            writeVideo(vid, frame);
        end
        if mod(i,25) == 0
            fprintf('progress: %d / %d\n', i, size(state_tree,1));
        end
    end
    
    % Visualize goal
    viscircles(goal, radius);
    if create_video
        frame = getframe(gcf);
        writeVideo(vid, frame);
    end
    
    % Visualize path to goal
    for i = 1:length
        index = path(i);
        curr_state = state_tree(index, 1:2);
        plot(curr_state(1), curr_state(2), '*r');
        delete(plot_array(1,i));
        % If it has a parent, plot a line
        if parents(index) ~= 0
            curr_parent = state_tree(parents(index),1:2);
            line([curr_state(1), curr_parent(1)], [curr_state(2), curr_parent(2)], 'Color', 'red');
            delete(plot_array(2,i));
        end
        if create_video
            frame = getframe(gcf);
            writeVideo(vid, frame);
        end
        if mod(i,25) == 0
            fprintf('progress: %d / %d\n', i, length);
        end
    end

    if create_video
        close(vid);
    end
end