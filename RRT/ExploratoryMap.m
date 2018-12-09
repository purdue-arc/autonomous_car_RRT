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
        observation_cuttoff
    end

    methods
        function obj = ExploratoryMap(x_min, x_max, y_min, y_max, scale, simple_map, vector_count, view_width, max_distance, observation_cuttoff)
            % Constructor
            obj = obj@Map(x_min, x_max, y_min, y_max, scale, simple_map);
            obj.observation_array = ones(size(obj.obstacle_array)) * 0.5; % Everything has weight of 0 to start
            obj.vector_count = vector_count;
            obj.view_width = view_width;
            obj.max_distance = max_distance;
            obj.observation_cutoff = observation_cutoff;
        end
        
        function knowledge = evaluate_state(obj, state)
            % Evaluate knowledge gained from a certain position with only existing knowledge
            % We don't know that hidden obstacles exist so we 'see' past them when predicting how much we will see
            [points_x, points_y, visibility] = obj.simulate_camera(state, false);
            knowledge = 0;
            for i=1:size(points_x)
                [row, col] = obj.get_rc_internal(points_x(i), points_y(i));
                current_vis = abs(0.5 - obj.observation_array(row, col)) * 2;
                new_vis = visibility(i);
                if new_vis > current_vis
                    knowledge = knowledge + new_vis - current_vis;
                end
            end
        end
        
        function execute_state(obj, state)
            [points_x, points_y, visibility] = obj.simulate_camera(state, true);
            for i=1:size(points_x)
                [row, col] = obj.get_rc_internal(points_x(i), points_y(i));
                current_obs = obj.observation_array(row, col);
                if map.get_cell_internal(points_x(i), points_y(i)) == 1
                    new_obs = 0.5 + 0.5 * visibility(i);
                else
                    new_obs = 0.5 * (1 - visibility(i));
                end
                if abs(.5 - new_obs) > abs(.5 - current_obs)
                    if abs(0.5 - new_obs) < obj.observation_cutoff
                        obj.observation_array(row, col) = round(new_obs);
                    else
                        obj.observation_array(row, col) = new_obs;
                    end
                end
            end
        end          
        
        function [visible_x, visible_y, visibility]  = simulate_camera(obj,state, stop_at_hidden_obstacle)
            % simCamera This simulates the view of a camera by creating a 2d cone of view based off of n vectors split accross a certain width
            
            % Location in internal units
            pos_x = state(1) * obj.scale;
            pos_y = state(2) * obj.scale;

            % figure out vectors' tails
            tails_x = zeros(5);
            tails_y = zeros(5);
            for v=1:obj.vector_count
                projection_angle = state(3) + (3-v) * obj.view_width / obj.vector_count; % Get the angle to compute
                for d=1:obj.max_distance*obj.scale
                    x = pos_x + d * cos(projection_angle);
                    y = pos_y + d * sin(projection_angle);
                    [row, col] = obj.get_rc_internal(x, y);
                    if stop_at_hidden_obstacle && obj.obstacle_array(row, col) == 1 || 
                        ~stop_at_hidden_obstacle && obj.observation_array(row, col) == 1
                        % Found an obstacle; terminate the vector
                        % Distance ends at wall, so round
                        tails_x(v) = round(x);
                        tails_y(v) = round(y);
                        break;
                    end
                    if d == obj.max_distance*obj.scale
                        % Didn't find a wall, but can no longer see
                        % Distance should be exact in this case, so don't round
                        tails_x(v) = x;
                        tails_y(v) = y;
                    end
                    % continue 'raycasting'
                end
            end

            % Create n-1 triangles out of these n vectors plus the starting point
            trianle_x = zeroes(4,3);
            triangle_y = zeroes(4,3);

            for v=1:(obj.vector_count-1)
                % Initial point
                trianle_x(v,1) = pos_x;
                triangle_y(v,1) = pos_y;
                % Left tail
                trianle_x(v,2) = tails_x(v);
                triangle_y(v,2) = tails_y(v);
                % Right tail
                trianle_x(v,3) = tails_x(v+1);
                triangle_y(v,3) = tails_y(v+1);
            end

            % Determine the set of points that we need to sample
            min_x = min(trianle_x, [],'all');
            max_x = max(trianle_x, [],'all');
            min_y = min(triangle_y, [],'all');
            max_y = max(triangle_y, [],'all');

            % Generate an array of these points to pass into the inpolygon function
            % Will be a box shape including all internal points + extras
            num_points = (max_x - min_x) * (max_y - min_y);
            box_x = zeros(num_points);
            box_y = zeros(num_points);

            dist_x = min_x;
            y = min_y;
            for i=1:num_points
                box_x(i) = dist_x;
                box_y(i) = y;
                dist_x = dist_x + 1;
                if dist_x == max_x
                    dist_x = min_x;
                    y = y + 1;
                end
            end

            % Figure out which of these points are visible (inside the generated triangles)
            visible_points = zeros(num_points);
            for v=1:obj.num_vectors
                visible_points = visible_points | inpolygon(box_x, box_y, transpose(trianle_x(v,:)), transpose(triangle_y(v,:)));
            end

            % Generate an array of just the internal points
            num_visible_points = 0;
            visible_points_index = []; % Index into the box_x and box_y arrays
            for i=1:num_points
                if visible_points(i) == 1
                    num_visible_points = num_visible_points + 1;
                    visible_points_index(num_visible_points) = i;
                end
            end

            visible_x = box_x(visible_points_index);
            visible_y = box_y(visible_points_index);

            % Figure out how well you see these points
            visibility = zeros(num_visible_points);
            for i=1:num_visible_points
                % Figure out distance
                dist_x = abs(visible_x(i) - pos_x);
                dist_y = abs(visible_y(i) - pos_y);
                dist = sqrt(dist_y^2 + dist_x^2);
                % Figure out visibility (1 == max, 0 = min)
                visibility(i) = max([1 - dist / obj.max_dist, 0]);
            end
        end
    end
end

