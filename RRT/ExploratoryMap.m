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
        explore_radius
    end

    methods
        function obj = ExploratoryMap(x_min, x_max, y_min, y_max, scale, simple_map, evaluation_vector_count, execution_vector_count, view_width, max_distance, obstacle_cutoff, explore_radius)
            % Constructor
            obj = obj@Map(x_min, x_max, y_min, y_max, scale, simple_map);
            obj.observation_array = ones(size(obj.obstacle_array)) * 0.5; % Everything has weight of 0 to start
            obj.evaluation_vector_count = evaluation_vector_count;
            obj.execution_vector_count = execution_vector_count;
            obj.view_width = view_width;
            obj.max_distance = max_distance;
            obj.obstacle_cuttoff = obstacle_cutoff;
            obj.explore_radius = explore_radius;
            
            % Used to determine the exploration progress
            obj.free_space = sum(~obj.obstacle_array, 'all');           
        end
        
        function knowledge = evaluate_state(obj, state)
            % Evaluate knowledge gained from a certain position with only existing knowledge
            % We don't know that hidden obstacles exist so we 'see' past them when predicting how much we will see
            % Rows for each point, col 1: x, col 2: y, col 3: visibility
            [points, vis] = obj.simulate_camera(state, false);
            knowledge = 0;
            for i=1:size(points, 1)
                [row, col] = obj.get_rc_internal(points(i,1), points(i,2));
                current_vis = abs(0.5 - obj.observation_array(row, col)) * 2;
                new_vis = vis(i);
                if new_vis > current_vis
                    knowledge = knowledge + new_vis - current_vis;
                end
            end
        end
        
        function view = execute_state(obj, state)
            % Rows for each point, col 1: x, col 2: y, col 3: visibility
            [points, vis] = obj.simulate_camera(state, true);
            for i=1:size(points, 1)
                [row, col] = obj.get_rc_internal(points(i,1), points(i,2));
                current_obs = obj.observation_array(row, col);
                % Figure out what the new observation is on a scale of 0 to 1
                if obj.get_cell_internal(points(i,1), points(i,2))
                    new_obs = 0.5 + 0.5 * vis(i);
                else
                    new_obs = 0.5 * (1 - vis(i));
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
            view = [double(points) / obj.scale, vis];
        end          
        
        function [visible_points, visible_points_vis] = simulate_camera(obj, state, execution)
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
            vector_end_points = int16(zeros(vector_count, 2));      % vector_count x 2 array for storing tail points
            projection_angles = linspace(state(3) - obj.view_width/2, state(3) + obj.view_width/2, vector_count);
            for v=1:vector_count
                [x_end, y_end] = obj.raycast(pos_x, pos_y, projection_angles(v), execution);
                vector_end_points(v,:) = [x_end, y_end];    % Stored as int, since internal units
            end
            
            % From now on we will use rounded position
            pos_x = int16(pos_x);
            pos_y = int16(pos_y);
           
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
            xs = repmat(min_x:max_x, 1, max_y-min_y+1);
            ys = repelem(min_y:max_y, max_x-min_x+1);
            box_points = [xs', ys'];

            % Figure out which of these points are visible (inside the generated n-gon)
            box_points_vis = inpolygon(box_points(:,1), box_points(:,2), polygon_x, polygon_y);

            % Generate an array of just the internal points
            visible_points = box_points(box_points_vis, :);  % col 1: x, col 2: y

            % Figure out how well you see these points
            dist = sqrt(double((visible_points(:,1) - pos_x).^2 + (visible_points(:,2) - pos_y).^2));
            visible_points_vis = max([1 - dist/(obj.scale*obj.max_distance), zeros(size(visible_points, 1),1)], [], 2);
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
        
        function [x_pos, y_pos] = gen_rand_explore_pos(obj, cur_x, cur_y)
            % Generate a random postion within the bounds
            % Override of Map class method that generates position in free space
            % Since we only see the edges, it isn't worth even making sure the new point is free
            % Also, should only generate within a certain radius of the vehicle to get more branched paths
            for i = 1:100 % try up to 100 times
                rand_radius = rand * obj.explore_radius;
                rand_angle = rand * 2 * pi;
                x_pos = cur_x + rand_radius * cos(rand_angle);
                y_pos = cur_y + rand_radius * sin(rand_angle);
                if obj.check_pos_explore(x_pos, y_pos)
                    return;
                end
                if i == 100
                    fprintf('Failed to generate random position\n');
                    return;
                end
            end
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

