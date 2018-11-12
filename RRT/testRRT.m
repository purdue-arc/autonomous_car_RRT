close all
clear variables

% Arbitrary values for the test
x_min = 0;
x_max = 10;
y_min = 0;
y_max = 10;

figure
hold on
state = [0.5, 0.5, pi/4, 0, 0]; % [x CG, y CG, theta, lateral speed(vy), yaw rate(r or thetadot)]

%tree(1,:) = cat(2, state, initial_x)
state_tree(1,:) = state;
parents = 0;

for i = 2:2500
    % Create a new random position in the map
    rand_pos = [rand(1) * (x_max - x_min) + x_min, rand(1) * (y_max - y_min) + y_min];
    
    % Pass this to extend function and add the resulting state to the array
    [state_tree, parents] = extend(state_tree, rand_pos, parents);
end

% Plot for debugging
for i = 1:size(state_tree, 1)
    curr_state = state_tree(i,:);
    plot(curr_state(1), curr_state(2), '*');
    % If it has a parent, plot a line
    if parents(i) ~= 0
        curr_parent = state_tree(parents(i),:);
        line([curr_state(1), curr_parent(1)], [curr_state(2), curr_parent(2)]);
    end
end


