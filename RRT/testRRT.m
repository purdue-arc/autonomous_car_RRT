close all
clear variables
%initial_x = [0 0];
figure
hold on
state = [1, 1, 0, 0, 0]; %x,y center of gravity, theta, lateral speed(vy), yaw rate(r or thetadot)
%tree(1,:) = cat(2, state, initial_x)
tree(1,:) = state;
children = 1;
for i = 0:1:20
    x_rand = [rand(1)*pi/3-pi/6 rand(1)*3];
    %x_rand = [rand(1)*60-30 rand(1)*3];
    [tree, children] = extend(tree, x_rand, children);
end
    
% for i = 1:1:9
%     tree(i,1)
%     tree(i,2)
%     plot(tree(i,1), tree(i,2),'*')
% end


