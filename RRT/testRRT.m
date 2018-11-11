close all
clear variables

% Arbitrary values for the test
x_min = -10;
x_max = 10;
y_min = -10;
y_max = 10;

figure
hold on
state = [0, 0, 0, 0, 0]; % [x CG, y CG, theta, lateral speed(vy), yaw rate(r or thetadot)]

%tree(1,:) = cat(2, state, initial_x)
tree(1,:) = state;
num_children = 1;

for i = 0:1:20
    % Create a new random position in the map
    pos_rand = [rand(x_max - x_min) + x_min, rand(y_max - y_min) + y_min];
    
    % Pass this to extend function and add the resulting state to the array
    [tree, num_children] = extend(tree, pos_rand, num_children);
end
    
% for i = 1:1:9
%     tree(i,1)
%     tree(i,2)
%     plot(tree(i,1), tree(i,2),'*')
% end


