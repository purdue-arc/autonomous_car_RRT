close all
clear variables

% Arbitrary values for the test
x_min = -2;
x_max = 2;
y_min = -2;
y_max = 2;

figure
hold on
state = [0, 0, 0, 0, 0]; % [x CG, y CG, theta, lateral speed(vy), yaw rate(r or thetadot)]

%tree(1,:) = cat(2, state, initial_x)
state_tree(1,:) = state;
parents = [0];

for i = 0:1:2500
    % Create a new random position in the map
    rand_pos = [rand(1) * (x_max - x_min) + x_min, rand(1) * (y_max - y_min) + y_min];
    
    %plot(rand_pos(1), rand_pos(2), "o");
    
    % Pass this to extend function and add the resulting state to the array
    [state_tree, parents] = extend(state_tree, rand_pos, parents);
    %pause(.25);
end
    
% for i = 1:1:9
%     tree(i,1)
%     tree(i,2)
%     plot(tree(i,1), tree(i,2),'*')
% end


