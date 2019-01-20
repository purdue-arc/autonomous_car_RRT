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
        observation_cutoff
    end

    methods
        function obj = ExploratoryMap(x_min, x_max, y_min, y_max, scale, simple_map, vector_count, view_width, max_distance, observation_cutoff)
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
            % Rows for each point, col 1: x, col 2: y, col 3: visibility
            visible_points = obj.simulate_camera(state, false);
            knowledge = 0;
            for i=1:size(visible_points, 1)
                [row, col] = obj.get_rc_internal(visible_points(i,1), visible_points(i,2));
                current_vis = abs(0.5 - obj.observation_array(row, col)) * 2;
                new_vis = visible_points(i,3);
                if new_vis > current_vis
                    knowledge = knowledge + new_vis - current_vis;
                end
            end
        end
        
        function visible_points = execute_state(obj, state)
            % Rows for each point, col 1: x, col 2: y, col 3: visibility
            visible_points = obj.simulate_camera(state, true);
            for i=1:size(visible_points, 1)
                [row, col] = obj.get_rc_internal(visible_points(i,1), visible_points(i,2));
                current_obs = obj.observation_array(row, col);
                % Figure out what the new observation is on a scale of 0 to 1
                if obj.get_cell_internal(visible_points(i,1), visible_points(i,2)) == 1
                    new_obs = 0.5 + 0.5 * visible_points(i,3);
                else
                    new_obs = 0.5 * (1 - visible_points(i,3));
                end
                obj.observation_array(row, col) = new_obs;
%                 end
%                 % If the new observation is more accurate . . .
%                 if abs(.5 - new_obs) > abs(.5 - current_obs)
%                     % If the observation is accurate enough, round it to 1 or 0
%                     if abs(0.5 - new_obs) < obj.observation_cutoff
%                         obj.observation_array(row, col) = round(new_obs);
%                     else
%                         obj.observation_array(row, col) = new_obs;
%                     end
%                 end
            end
        end          
        
        function visible_points = simulate_camera(obj, state, stop_at_hidden_obstacle)
            % simCamera This simulates the view of a camera by creating a 2d triangle of view based off of n vectors split accross a certain width
            
            % Location in internal units
            pos_x = state(1) * obj.scale;
            pos_y = state(2) * obj.scale;

            % figure out vectors' tails
            vector_end_points = zeros(5,2);   % 5x2 array for storing tail points
            for v=1:obj.vector_count
                % TODO remove hardcoded 3
                projection_angle = state(3) + (ceil(obj.vector_count/2)-v) * obj.view_width / obj.vector_count; % Get the angle to compute
                for d=1:obj.max_distance*obj.scale
                    % Internal position of end of ray
                    x = round(pos_x + d * cos(projection_angle));
                    y = round(pos_y + d * sin(projection_angle));
                    % Check that [x,y] is within bounds of map
                    if x >= obj.x_min*obj.scale && x < obj.x_max*obj.scale && y >= obj.y_min*obj.scale && y < obj.y_max*obj.scale
                        [row, col] = obj.get_rc_internal(x, y);
                        if stop_at_hidden_obstacle && obj.obstacle_array(row, col) == 1 || ...
                            ~stop_at_hidden_obstacle && obj.observation_array(row, col) == 1
                            % Found an obstacle; terminate the vector
                            vector_end_points(v,:) = [x,y];
                            break;
                        end
                        if d == obj.max_distance*obj.scale
                            % Didn't find a wall, but can no longer see
                            vector_end_points(v,:) = [x,y];
                        end
                        % continue 'raycasting'
                    else
                        % Went outside bounds, end at last point
                        x = round(pos_x + (d-1) * cos(projection_angle));
                        y = round(pos_y + (d-1) * sin(projection_angle));
                        vector_end_points(v,:) = [x,y];
                        break;
                    end
                end
            end

            % Create n-1 triangles out of these n vectors plus the starting point
            triangle_x = zeros(4,3);
            triangle_y = zeros(4,3);

            for v=1:(obj.vector_count-1)
                % Initial point
                triangle_x(v,1) = pos_x;
                triangle_y(v,1) = pos_y;
                % Left tail
                triangle_x(v,2) = vector_end_points(v, 1);
                triangle_y(v,2) = vector_end_points(v, 2);
                % Right tail
                triangle_x(v,3) = vector_end_points(v+1, 1);
                triangle_y(v,3) = vector_end_points(v+1, 2);
            end

            % Determine the boundaries of the points that we need to sample
            % These points form a big box including all points
            min_x = min(triangle_x, [], 'all');
            max_x = max(triangle_x, [], 'all');
            min_y = min(triangle_y, [], 'all');
            max_y = max(triangle_y, [], 'all');

            % Generate an array of these points to pass into the inpolygon function
            num_points = (max_x - min_x) * (max_y - min_y);
            box_points = zeros(num_points,3);   % col 1: x, col 2: y, col 3: internal

            % Populate this array with points
            x = min_x;
            y = min_y;
            for i=1:num_points
                box_points(i, 1) = x;
                box_points(i, 2) = y;
                x = x + 1;
                if x == max_x
                    x = min_x;
                    y = y + 1;
                end
            end

            % Figure out which of these points are visible (inside the generated triangles)
            for v=1:(obj.vector_count-1)
                box_points(:,3) = box_points(:,3) | inpolygon(box_points(:,1), box_points(:,2), transpose(triangle_x(v,:)), transpose(triangle_y(v,:)));
            end

            % Generate an array of just the internal points
            visible_points = box_points(find(box_points(:,3)), :);  % col 1: x, col 2: y, col 3: vis (currently has 1)

            % Figure out how well you see these points
            for i=1:size(visible_points, 1)
                % Figure out distance
                dist_x = abs(visible_points(i, 1) - pos_x);
                dist_y = abs(visible_points(i, 2) - pos_y);
                dist = sqrt(dist_y^2 + dist_x^2);
                % Figure out visibility (1 == max, 0 = min)
                visible_points(i, 3) = max([1 - dist / (obj.max_distance * obj.scale), 0]);
            end
        end
    end
end

