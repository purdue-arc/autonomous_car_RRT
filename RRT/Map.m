classdef Map < handle
    % This class handles related functions for keeping an array based 2D map of the environment
    
    properties
        obstacle_array
        x_min
        x_max
        y_min
        y_max
        scale
    end
    
    methods
        function obj = Map(x_min, x_max, y_min, y_max, scale, simple_map)
            % Construct an instance of this class
            % Pass in mins and maxes in world units (typ. meters)
            % Pass in scale (ex: 10 results in array where each cell in the array is .1 world units by .1 world units
            % Pass in simple map that will be scaled up (scaling up must be equal along both axis) (must be smaller than the new map)
            
            obj.x_min = x_min;
            obj.x_max = x_max;
            obj.y_min = y_min;
            obj.y_max = y_max;
            obj.scale = scale;
            
            resize_factor = (y_max - y_min) * scale / (size(simple_map, 1));
            obj.obstacle_array = imresize(simple_map, resize_factor, 'nearest');
        end
        
        function valid = check_pos(obj, x_pos, y_pos)
            % Check if an x,y position is a valid position on the map (within bounds & obstacle free)
            valid = x_pos >= obj.x_min && x_pos < obj.x_max && y_pos >= obj.y_min && y_pos < obj.y_max && ~obj.get_cell(x_pos, y_pos);
            return;
        end
        
        function [x_pos, y_pos] = gen_rand_pos(obj)
            % Generate a random postion within the bounds and free of obstacles
            
            for i = 1:100 % try up to 100 times
                x_pos = rand * (obj.x_max - obj.x_min) + obj.x_min;
                y_pos = rand * (obj.y_max - obj.y_min) + obj.y_min;

                if obj.check_pos(x_pos, y_pos)
                    return;
                end
                if i == 100
                    fprintf('Failed to generate random position\n');
                    return;
                end
            end
        end
        
        function cell = get_cell(obj, x_pos, y_pos)
            % Returns the value of a cell in world coordinates
            cell = obj.get_cell_internal(x_pos*obj.scale, y_pos*obj.scale);
        end
        
        function cell = get_cell_internal(obj, x_pos, y_pos)
            % Returns the value of a cell in internal coordinates
            [row, col] = obj.get_rc_internal(x_pos, y_pos);
            cell = obj.obstacle_array(row, col);
        end
        
        function [row, col] = get_rc_internal(obj, x_pos, y_pos)
            % Returns the rc position of a cell in internal coordinates
            col = floor(x_pos) + 1;
            row = obj.y_max * obj.scale - floor(y_pos);
        end
    end
end

