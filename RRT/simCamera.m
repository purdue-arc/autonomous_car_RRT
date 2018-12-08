function [new_map, gained_knowledge] = simCamera(observed_map, map, state)
    % simCamera This simulates the view of a camera by creating a 2d cone of
    % view based off of n vectors split accross a certain width

    vector_count = 5;
    view_width = pi/3; % radians (60 degrees)
    max_distance = 10; % meters

    % figure out vectors' tails
    vector_x = zeros(5);
    vector_y = zeros(5);
    for v=1:vector_count
        projection_angle = state(3) + (3-v) * view_width/vector_count; % Get the angle to compute
        for d=1:max_distance*map.scale
            x_pos = d * cos(projection_angle) / map.scale;
            y_pos = d * sin(projection_angle) / map.scale;
            cell = map.getCell(x_pos, y_pos);
            if cell == 1
                % Found an obstacle; terminate
                vector_x(v) = round(x_pos, log10(map.scale));
                vector_y(v) = round(y_pos, log10(map.scale));
                break;
            end
            if d == max_distance
                % Didn't find a wall, but can no longer see
                vector_x(v) = round(x_pos, log10(map.scale));
                vector_y(v) = round(y_pos, log10(map.scale));
                break; % Explicit break
            end
            % Otherwise continue
        end
    end

    % Create 4 triangles out of these five vectors plus the starting point
    triangle_x = zeroes(4,3);
    triangle_y = zeroes(4,3);

    for v=1:(vector_count-1)
        % Initial point
        triangle_x(v,1) = round(state(1), log10(map.scale));
        triangle_y(v,1) = round(state(2), log10(map.scale));
        % Left tail
        triangle_x(v,2) = vector_x(v);
        triangle_y(v,2) = vector_y(v);
        % Right tail
        triangle_x(v,3) = vector_x(v+1);
        triangle_y(v,3) = vector_y(v+1);
    end

    % Determine the set of points that we need to sample
    min_x = min(triangle_x, [],'all');
    max_x = max(triangle_x, [],'all');
    min_y = min(triangle_y, [],'all');
    max_y = max(triangle_y, [],'all');

    % Generate an array of these points to pass into the inpolygon function
    num_x_points = (max_x - min_x) * map.scale;
    num_y_points = (max_y - min_y) * map.scale;
    num_points = num_x_points * num_y_points;
    point_x = zeros(num_points);
    point_y = zeros(num_points);

    x = min_x;
    y = min_y;
    for i=1:num_points
        point_x(i) = x;
        point_y(i) = y;
        x = x + 1/map.scale;
        if x == max_x
            x = min_x;
            y = y + 1/map.scale;
        end
    end

    % Figure out which of these points are inside the different triangles
    internal_points = zeros(num_points);
    for v=1:num_vectors
        internal_points = internal_points | inpolygon(point_x, point_y, transpose(triangle_x(v,:)), transpose(triangle_y(v,:)));
    end

    % Generate an array of just the points you can 'see'
    num_internal_points = 0;
    internal_points_index = [];
    for i=1:num_points
        if internal_points(i) == 1
            num_internal_points = num_internal_points + 1;
            internal_points_index(num_internal_points) = i;
        end
    end

    internal_points_x = points_x(internal_points_index);
    internal_points_y = points_y(internal_points_index);

    % Figure out what you observe of these points
    internal_points_obs = zeros(num_internal_points);
    for i=1:num_internal_points
        % Figure out distance
        dist_x = abs(internal_points_x(i) - state(1));
        dist_y = abs(internal_points_y(i) - state(2));
        dist = sqrt(dist_y^2 + dist_x^2);
        % Figure out visibility (1 == max, 0 = min)
        visibility = max([1 - dist / max_dist, 0]);
        % Figure out observation
        internal_points_obs = ~map.get_cell(internal_points_x(i), internal_points_y(i)) - visibility;
    end
    
    % Merge this in with existing observation?
end


