% This file is sort of a sandbox for running different tests

% Optimizations
% imresize takes slightly longer but only runs once and is foolproof / cleaner
% Positions should be rounded
% vectorization?
% gpu array?
% arrayfun?
% logical indexing
% types
% use inpolygon better

% Current Bugs:
% raycast ends are all over place - ensure pre-allocated array is correct size * init pos is in right units
% Large sizing - use units properly
% Additional - linspace for calculating raycast angles

%% testing creating array of points to sample

% original
min_x = 0;
min_y = 0;
max_x = 100;
max_y = 100;

num_points = (max_x - min_x+1) * (max_y - min_y+1);
box_points = zeros(num_points, 2, 'int16');             % col 1: x, col 2: y

% Populate this array with points
x = min_x;
y = min_y;
for i=1:num_points
    box_points(i, 1) = x;
    box_points(i, 2) = y;
    x = x + 1;
    if x > max_x
        x = min_x;
        y = y + 1;
    end
end
xs = repmat(min_x:max_x, 1, max_y-min_y+1);
ys = repelem(min_y:max_y, max_x-min_x+1);
box_points_new = [xs', ys'];

diff = box_points ~= box_points_new;
diff_index = find(diff);

%% other

filename = "100_map.mat";
mat = matfile(filename);
simple_map = mat.obstacle_matrix;

array2 = create_array_img(simple_map, 50, 10);

[cur_view, cur_view_vis] = simulate_camera(array2, 10, [.5, .5, deg2rad(45)], true, 91);

set(gcf, 'Position', [300 200 1280 720]);
colormap(flipud(gray));
title("Obstacle Map");
    xlabel("X Position (m)");
    ylabel("Y Position (m)");
    hold on;
    axis([0 50 0 50], 'square');                  % Set axis
                                                                % Plot image
imagesc('XData', [0.5/10,  50 - 0.5/10], 'YData', [50 - 0.5/10,  0.5/10], 'CData', array2);
cmap = flipud(autumn(100));
color_index = uint8(cur_view_vis * 99) + 1;
colors = cmap(color_index, :);
scatter(double(cur_view(:,1))/10, double(cur_view(:,2))/10, 10, colors);       % Visibility

end_view = zeros(91,2, 'uint8');
for i = 0:90
    angle = deg2rad(i);
    [x_end, y_end] = raycast(array2, 0.5 * 10, 0.5 * 10, angle, true);
    end_view(i+1,:) = [x_end, y_end];
end

end_view = double(end_view) / 10;

plot(end_view(:,1), end_view(:,2), 'r-');       % Visibility

polygon_x = [0.5, end_view(:,1)', 0.5]';
polygon_y = [0.5, end_view(:,2)', 0.5]';

plot(polygon_x, polygon_y, 'k-');       % Visibility

function obstacle_array = create_array(simple_map, size, scale)
    obstacle_array = zeros(size * scale);
    org_size = 100;
    adj = size * scale / org_size;
    % Need to populate the true map with values 
    for i=1:100
        for j=1:100
            % Copy data to relevant blocks
            if simple_map(i,j) == 1
                for k=(1 + adj*(i-1)):(adj + adj*(i-1))
                    for l=(1 + adj*(j-1)):(adj + adj*(j-1))
                        obstacle_array(k,l) = 1;
                    end
                end
            end
        end
    end
end

function obstacle_array = create_array_img(simple_map, size, scale)
    adj = size * scale / 100;
    obstacle_array = imresize(simple_map, adj, 'nearest');
end

function [visible_points, visible_points_vis] = simulate_camera(map, scale, state, execution, vector_count)
            % simCamera This simulates the view of a camera by creating a 2d triangle of view based off of n vectors split accross a certain width
        
            % Location in internal units
            pos_x = state(1) * scale;
            pos_y = state(2) * scale;

            % figure out vectors' tails
            % Create column vectors of x and y
            vector_end_points = zeros(vector_count, 2, 'int16');   % vector_count x 2 array for storing tail points
            angles = linspace(state(3) - deg2rad(45), state(3) + deg2rad(45), vector_count);
            for v = 1:vector_count
                [x_end, y_end] = raycast(map, pos_x, pos_y, angles(v), execution);
                vector_end_points(v,:) = [x_end, y_end];
            end

%             % Create n-1 triangles out of these n vectors plus the starting point
%             triangle_x = zeros(vector_count-1,3, 'int16');
%             triangle_y = zeros(vector_count-1,3, 'int16');
% 
%             for a=1:(vector_count-1)
%                 % Initial point
%                 triangle_x(a,1) = pos_x;
%                 triangle_y(a,1) = pos_y;
%                 % Left tail
%                 triangle_x(a,2) = vector_end_points(a, 1);
%                 triangle_y(a,2) = vector_end_points(a, 2);
%                 % Right tail
%                 triangle_x(a,3) = vector_end_points(a+1, 1);
%                 triangle_y(a,3) = vector_end_points(a+1, 2);
%             end

            % Create a really big n-gon of the base point and all the end points
            polygon_x = int16([pos_x, vector_end_points(:,1)']');
            polygon_y = int16([pos_y, vector_end_points(:,2)']');

            % Determine the boundaries of the points that we need to sample
            % These points form a big box including all points
            min_x = min(polygon_x, [], 'all');
            max_x = max(polygon_x, [], 'all');
            min_y = min(polygon_y, [], 'all');
            max_y = max(polygon_y, [], 'all');

            % Generate an array of these points to pass into the inpolygon function
            num_points = (max_x - min_x) * (max_y - min_y);
            box_points = zeros(num_points, 2, 'int16');  % col 1: x, col 2: y
            box_points_vis = zeros(num_points, 1, 'logical');

            % Populate this array with points
            x = min_x;
            y = min_y;
            for i=1:num_points
                box_points(i, 1) = x;
                box_points(i, 2) = y;
                x = x + 1;
                if x >= max_x
                    x = min_x;
                    y = y + 1;
                end
            end

            % Figure out which of these points are visible (inside the generated triangles)
            
            box_points_vis = inpolygon(box_points(:,1), box_points(:,2), polygon_x, polygon_y);
            
%             for a=1:(vector_count-1)
%                 box_points_vis = box_points_vis | inpolygon(box_points(:,1), box_points(:,2), triangle_x(a,:)', triangle_y(a,:)');
%             end

            % Generate an array of just the internal points
            visible_points = box_points(box_points_vis, :);  % col 1: x, col 2: y, col 3: vis (currently has 1)
            visible_points_vis = zeros(sum(box_points_vis), 1);
            
            % Figure out how well you see these points
            dist = sqrt(double((visible_points(:,1) - pos_x).^2 + visible_points(:,2).^2));
            visible_points_vis = max([1 - dist/(scale*10), zeros(size(visible_points, 1),1)], [], 2);
            
            
            
            
%             for i=1:size(visible_points, 1)
%                 % Figure out distance
%                 dist_x = visible_points(i, 1) - pos_x;
%                 dist_y = visible_points(i, 2) - pos_y;
%                 dist = sqrt(dist_y^2 + dist_x^2);
%                 % Figure out visibility (1 == max, 0 = min)
%                 visible_points_vis = max([1 - dist / (10* scale), 0]);
%             end
        end
        
function [x_end, y_end] = raycast(map, pos_x, pos_y, projection_angle, execution)
    % Raycasts on either the obstacle or observation map in order to simulate a camera's view
    % Operates using internal units
    max_distance = 10;
    scale = 10;
    
    fprintf("angle is: %g\n", rad2deg(projection_angle));
    
    %path = zeros(max_distance * scale,2);

    % Step through each point along the ray, using the internal scale so we don't miss any cells
    for d=1:max_distance * scale
        % Internal position of end of ray
        x = round(pos_x + d * cos(projection_angle)); % Round here
        y = round(pos_y + d * sin(projection_angle));

        % Check that [x,y] is within bounds of map
        if x >= 0 && x < 50*scale && y >+ 0 && y < 50*scale

            % Check if we hit an obstacle (using the relevant map)
            col = floor(x) + 1;
            row = 50 * scale - floor(y);
            if (execution && map(row, col) == 1) || (~execution && map(row, col) == 1)
                % Found an obstacle; terminate the vector here (so we detect the obstacle)
                x_end = x;
                y_end = y;
                break;
            end

            % Check if we hit the max distance
            if d == max_distance * scale
                % Didn't find a wall, but can no longer see
                x_end = x;
                y_end = y;
            end
            % Continue raycasting otherwise
        else
            % Went outside bounds, end at last point
            x_end = round(pos_x + (d-1) * cos(projection_angle));   % and here
            y_end = round(pos_y + (d-1) * sin(projection_angle));
            break;
        end
    end
end
