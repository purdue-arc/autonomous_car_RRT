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
        evaluation_vector_count
        execution_vector_count
        view_width
        max_distance
        obstacle_cuttoff
        free_space
    end

    methods
        function obj = ExploratoryMap(x_min, x_max, y_min, y_max, scale, simple_map, evaluation_vector_count, execution_vector_count, view_width, max_distance, obstacle_cutoff)
            % Constructor
            obj = obj@Map(x_min, x_max, y_min, y_max, scale, simple_map);
            obj.observation_array = ones(size(obj.obstacle_array)) * 0.5; % Everything has weight of 0 to start
            obj.evaluation_vector_count = evaluation_vector_count;
            obj.execution_vector_count = execution_vector_count;
            obj.view_width = view_width;
            obj.max_distance = max_distance;
            obj.obstacle_cuttoff = obstacle_cutoff;
            
            % Used to determine the exploration progress
            obj.free_space = sum(~obj.obstacle_array, 'all');           
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
%             scaled_knowledge = knowledge / obj.scale^2 / obj.max_knowledge;
        end
        
        function view = execute_state(obj, state)
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
                % If the new observation is more accurate . . .
                if abs(.5 - new_obs) > abs(.5 - current_obs)
                    % If the observation is accurate enough, round it to 1
                    if new_obs > obj.obstacle_cuttoff
                        obj.observation_array(row, col) = 1;
                    else
                        obj.observation_array(row, col) = new_obs;
                    end
                end
            end
            view = visible_points./[obj.scale, obj.scale, 1];
        end          
        
        function visible_points = simulate_camera(obj, state, execution)
            % simCamera This simulates the view of a camera by creating a 2d triangle of view based off of n vectors split accross a certain width
            if execution
                vector_count = obj.execution_vector_count;
            else
                vector_count = obj.evaluation_vector_count;
            end
            % Location in internal units
            pos_x = state(1) * obj.scale;
            pos_y = state(2) * obj.scale;

            % figure out vectors' tails
            vector_end_points = zeros(vector_count, 2);   % vector_count x 2 array for storing tail points
            projection_angles = linspace(state(3) - obj.view_width/2, state(3) + obj.view_width/2, vector_count);
            for v=1:vector_count
                [x_end, y_end] = obj.raycast(pos_x, pos_y, projection_angles(v), execution);
                vector_end_points(v,:) = [x_end, y_end];
            end
            
            % From now on we will use rounded position
            pos_x = round(pos_x);
            pos_y = round(pos_y);

            % Create n-1 triangles out of these n vectors plus the starting point
            triangle_x = zeros(vector_count-1,3);
            triangle_y = zeros(vector_count-1,3);

            for v=1:(vector_count-1)
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
            box_points = zeros(num_points, 3);  % col 1: x, col 2: y, col 3: internal

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
            for v=1:(vector_count-1)
                box_points(:,3) = box_points(:,3) | inpolygon(box_points(:,1), box_points(:,2), transpose(triangle_x(v,:)), transpose(triangle_y(v,:)));
            end

            % Generate an array of just the internal points
            visible_points = box_points(find(box_points(:,3)), :);  % col 1: x, col 2: y, col 3: vis (currently has 1)

            % Figure out how well you see these points
            for i=1:size(visible_points, 1)
                % Figure out distance
                dist_x = visible_points(i, 1) - pos_x;
                dist_y = visible_points(i, 2) - pos_y;
                dist = sqrt(dist_y^2 + dist_x^2);
                % Figure out visibility (1 == max, 0 = min)
                visible_points(i, 3) = max([1 - dist / (obj.max_distance * obj.scale), 0]);
            end
        end
        
        function [x_end, y_end] = raycast(obj, pos_x, pos_y, projection_angle, execution)
            % Raycasts on either the obstacle or observation map in order to simulate a camera's view
            % Operates using internal units
            
            % Step through each point along the ray, using the internal scale so we don't miss any cells
            for d=1:obj.max_distance * obj.scale
                % Internal position of end of ray
                x = round(pos_x + d * cos(projection_angle)); % Round here
                y = round(pos_y + d * sin(projection_angle));

                % Check that [x,y] is within bounds of map
                if x >= obj.x_min*obj.scale && x < obj.x_max*obj.scale && y >= obj.y_min*obj.scale && y < obj.y_max*obj.scale

                    % Check if we hit an obstacle (using the relevant map)
                    [row, col] = obj.get_rc_internal(x, y);
                    if (execution && obj.obstacle_array(row, col) == 1) || (~execution && obj.observation_array(row, col) == 1)
                        % Found an obstacle; terminate the vector here (so we detect the obstacle)
                        x_end = x;
                        y_end = y;
                        break;
                    end
                    
                    % Check if we hit the max distance
                    if d == obj.max_distance * obj.scale
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
        
        function [x_pos, y_pos] = gen_rand_pos(obj)
            % Generate a random postion within the bounds
            % Override of Map class method that generates position in free space
            % Since we only see the edges, it isn't worth even making sure the new point is free
            x_pos = rand * (obj.x_max - obj.x_min) + obj.x_min;
            y_pos = rand * (obj.y_max - obj.y_min) + obj.y_min;
        end
        
        function valid = check_pos_explore(obj, x_pos, y_pos)
            % Check if an x,y position is a valid position on the map (within bounds and free on observed map)
            [row, col] = obj.get_rc_internal(x_pos*obj.scale, y_pos*obj.scale);
            valid = x_pos >= obj.x_min && x_pos < obj.x_max && y_pos >= obj.y_min && y_pos < obj.y_max && obj.observation_array(row, col) ~= 1;
            return;
        end         
        
        function progress = get_exploration_progress(obj)
            % Determine what percentage of the obstacle free map has been explored
            observation = sum(~obj.obstacle_array .* (0.5 - obj.observation_array) * 2, 'all');
            progress = observation / obj.free_space;
        end
    end
end

