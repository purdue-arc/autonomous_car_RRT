clear all;
close all;

% Generate an m by n binary occupancy grid where the obstaces are clustered into islands or other shapes as opposed to being purely randomized

n_rows = 100;
n_cols = 100;
initial_obstacles = 0.95;
min_obstacles = 0.075;
max_obstacles = 0.75;
n_free = 2;
filename = "100_map.mat";

rand_matrix = rand(n_rows, n_cols);

subplot(2,2,1);
imshow(1-rand_matrix, 'InitialMagnification', 'fit');

obstacle_matrix_read = rand_matrix >= initial_obstacles;
obstacle_matrix_mod = obstacle_matrix_read;

subplot(2,2,3);
imshow(~obstacle_matrix_read, 'InitialMagnification', 'fit');
%free = find(rand_matrix < 0.25);

[r, c] = find((rand_matrix >= min_obstacles) & (rand_matrix < max_obstacles));
possible_obstacles = [r, c];

neighbors = [0 1;
             0 -1;
             1 0;
            -1 0
             1 1
             1 -1
            -1 1
            -1 -1];

for i=1:size(possible_obstacles)
    row = possible_obstacles(i,1);
    col = possible_obstacles(i,2);
	num_neighbors = 0;
    for j=1:size(neighbors,1)
        r = neighbors(j,1) + row;
        c = neighbors(j,2) + col;
        if r > 0 && r <= n_rows && c > 0 && c <= n_cols
            % we can check if this is good
            num_neighbors = num_neighbors + obstacle_matrix_read(r,c);
        end
    end
    if num_neighbors/size(neighbors,1) >= rand_matrix(row, col)
        obstacle_matrix_mod(row, col) = 1;
    end
end

subplot(1,2,2);
imshow(~obstacle_matrix_mod, 'InitialMagnification', 'fit');

obstacle_matrix_mod(1:n_free, 1:n_free) = zeros(n_free);
save(filename, "obstacle_matrix_mod");