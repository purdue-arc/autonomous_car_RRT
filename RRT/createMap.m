clear all;
close all;

% Generate an m by n binary occupancy grid where the obstaces are clustered into islands or other shapes as opposed to being purely randomized

n_rows = 100;
n_cols = 100;
initial_obstacles = 1-50/10000;
branch_probability = 0.90;
n_free = 2;
filename = "100_map.mat";

rand_matrix = rand(n_rows, n_cols);

subplot(2,2,1);
imshow(1-rand_matrix, 'InitialMagnification', 'fit');

obstacle_matrix = rand_matrix >= initial_obstacles;
[obstacle_rows, obstacle_cols] = find(obstacle_matrix);

subplot(2,2,3);
imshow(~obstacle_matrix, 'InitialMagnification', 'fit');

neighbors = [0, 1;
             0, -1;
             1, 0;
            -1, 0;
             1, 1;
             1, -1;
            -1, 1;
            -1, -1];

for i=1:size(obstacle_rows)
    cur_row = obstacle_rows(i);
    cur_col = obstacle_cols(i);
    while rand <= branch_probability
        rand_neighbor = round(rand * (size(neighbors,1)-1) + 1);
        delta_row = neighbors(rand_neighbor, 1);
        delta_col = neighbors(rand_neighbor, 2);
        
        new_row = delta_row + cur_row;
        new_col = delta_col + cur_col;
        
        % Check if it is valid
        if new_row > 0 && new_row <= n_rows && new_col > 0 && new_col <= n_cols
            % New location is valid, add it in then build off of it
            obstacle_matrix(new_row, new_col) = 1;
            cur_row = new_row;
            cur_col = new_col;
            % Loop through again
        else
            % Looks like we found a dead end, go to next possible obstacle
            break;
        end
    end

end

subplot(1,2,2);
imshow(~obstacle_matrix, 'InitialMagnification', 'fit');

obstacle_matrix(1:n_free, 1:n_free) = zeros(n_free);
save(filename, "obstacle_matrix");