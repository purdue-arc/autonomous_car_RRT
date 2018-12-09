classdef ExploratoryMap < Map
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % From superclass:
        %  obstacle_array
        %  x_min
        %  x_max
        %  y_min
        %  y_max
        %  scale
        observation_array
        vector_count
        view_width
        max_distance
    end
    
    methods
        function obj = ExploratoryMap(x_min, x_max, y_min, y_max, scale, simple_map, vector_count, view_width, max_distance)
            % Constructor
            obj = obj@Map(x_min, x_max, y_min, y_max, scale, simple_map);
            obj.observation_array = zeros(size(obj.obstacle_array));
            obj.vector_count = vector_count;
            obj.view_width = view_width;
            obj.max_distance = max_distance;
        end
        
        function [visible_points_x_internal, visible_points_y_internal, visible_points_observation]  = simulate_camera(obj,state)
            % simCamera This simulates the view of a camera by creating a 2d cone of view based off of n vectors split accross a certain width
            
            % Location in internal units
            pos_x_internal = state(1) * obj.scale;
            pos_y_internal = state(2) * obj.scale;

            % figure out vectors' tails
            vector_x_internal = zeros(5);
            vector_y_internal = zeros(5);
            for v=1:obj.vector_count
                projection_angle = state(3) + (3-v) * obj.view_width/obj.vector_count; % Get the angle to compute
                for d=1:obj.max_distance*obj.scale
                    x_internal = pos_x_internal + d * cos(projection_angle);
                    y_internal = pos_y_internal + d * sin(projection_angle);
                    cell = obj.get_cell_internal(x_internal, y_internal);
                    if cell == 1
                        % Found an obstacle; terminate the vector
                        % Distance ends at wall, so round
                        vector_x_internal(v) = round(x_internal);
                        vector_y_internal(v) = round(y_internal);
                        break;
                    end
                    if d == obj.max_distance*obj.scale
                        % Didn't find a wall, but can no longer see
                        % Distance should be 10m, so don't round
                        vector_x_internal(v) = x;
                        vector_y_internal(v) = y;
                        break; % Explicit break
                    end
                    % Otherwise continue 'raycasting'
                end
            end

            % Create n-1 triangles out of these n vectors plus the starting point
            triangle_x_internal = zeroes(4,3);
            triangle_y_internal = zeroes(4,3);

            for v=1:(obj.vector_count-1)
                % Initial point
                triangle_x_internal(v,1) = pos_x_internal;
                triangle_y_internal(v,1) = pos_y_internal;
                % Left tail
                triangle_x_internal(v,2) = vector_x_internal(v);
                triangle_y_internal(v,2) = vector_y_internal(v);
                % Right tail
                triangle_x_internal(v,3) = vector_x_internal(v+1);
                triangle_y_internal(v,3) = vector_y_internal(v+1);
            end

            % Determine the set of points that we need to sample
            min_x_internal = min(triangle_x_internal, [],'all');
            max_x_internal = max(triangle_x_internal, [],'all');
            min_y_internal = min(triangle_y_internal, [],'all');
            max_y_internal = max(triangle_y_internal, [],'all');

            % Generate an array of these points to pass into the inpolygon function
            % Will be a box shape including all internal points + extras
            num_x_points = (max_x_internal - min_x_internal);
            num_y_points = (max_y_internal - min_y_internal);
            num_points = num_x_points * num_y_points;
            box_x_internal = zeros(num_points);
            box_y_internal = zeros(num_points);

            x = min_x_internal;
            y = min_y_internal;
            for i=1:num_points
                box_x_internal(i) = x;
                box_y_internal(i) = y;
                x = x + 1;
                if x == max_x_internal
                    x = min_x_internal;
                    y = y + 1;
                end
            end

            % Figure out which of these points are visible (inside the generated triangles)
            visible_points = zeros(num_points);
            for v=1:obj.num_vectors
                visible_points = visible_points | inpolygon(box_x_internal, box_y_internal, transpose(triangle_x_internal(v,:)), transpose(triangle_y_internal(v,:)));
            end

            % Generate an array of just the internal points
            num_visible_points = 0;
            visible_points_index = [];
            for i=1:num_points
                if visible_points(i) == 1
                    num_visible_points = num_visible_points + 1;
                    visible_points_index(num_visible_points) = i;
                end
            end

            visible_points_x_internal = box_x_internal(visible_points_index);
            visible_points_y_internal = box_y_internal(visible_points_index);

            % Figure out what you observe of these points
            visible_points_observation = zeros(num_visible_points);
            for i=1:num_visible_points
                % Figure out distance
                dist_x_internal = abs(visible_points_x_internal(i) - pos_x_internal);
                dist_y_internal = abs(visible_points_y_internal(i) - pos_y_internal);
                dist_internal = sqrt(dist_y_internal^2 + dist_x_internal^2);
                % Figure out visibility (1 == max, 0 = min)
                visibility = max([1 - dist_internal / obj.max_dist, 0]);
                % Figure out observation
                visible_points_observation = ~map.get_cell_internal(visible_points_x_internal(i), visible_points_y_internal(i)) - visibility;
            end
        end
    end
end

