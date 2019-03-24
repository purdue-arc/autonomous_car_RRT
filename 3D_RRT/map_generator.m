% This will generate a map to be used and display it in 3D

% Lets start out by trying peaks
heightmap = peaks(100);

% From this heightmap, lets make a bunch of squares


%lets do an example
% fill3([0;0;0;0;], [1;1;1;1;], [1;1;1;1), 

% X = [0 1 1 2;
%      1 1 2 2;
%      0 0 1 1];
% 
% Y = [1 1 1 1;
%      1 0 1 0;
%      0 0 0 0];
% 
% Z = [1 1 1 1;
%      1 0 1 0;
%      0 0 0 0];
% 
% C = [0.5000 1.0000 1.0000 0.5000;
%      1.0000 0.5000 0.5000 0.1667;
%      0.3330 0.3330 0.5000 0.5000];
% 
% figure
% view(3)
% patch(X,Y,Z,'b')


% test with quads
X = [0;1;1;0];
Y = [0;0;1;1];
Z = [0;0;1;1];

figure
view(3)
fill3(X,Y,Z,'b');