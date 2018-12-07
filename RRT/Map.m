classdef Map
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
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
            
            obj.obstacle_array = zeros((y_max - y_min) * scale, (x_max - x_min) * scale);
            r_scale = size(obj.obstacle_array, 1) / size(simple_map, 1);
            c_scale = size(obj.obstacle_array, 2) / size(simple_map, 2);
            
            % Need to populate the true map with values 
            for i=1:size(simple_map, 1)
                for j=1:size(simple_map, 2)
                    % Copy data to relevant blocks
                    if simple_map(i,j) == 1
                        for k=(1 + r_scale*(i-1)):(r_scale + r_scale*(i-1))
                            for l=(1 + c_scale*(j-1)):(c_scale + c_scale*(j-1))
                                obj.obstacle_array(k,l) = 1;
                            end
                        end
                    end
                end
            end
        end
        
        function valid = check_pos(obj, x_pos, y_pos)
            % Check if an x,y position is a valid position on the map (within bounds & obstacle free)
            
            % Check bounds
            if x_pos > obj.x_min && x_pos < obj.x_max && y_pos > obj.y_min && y_pos < obj.y_max
                % continue to check for obstacle
                col = floor(x_pos * obj.scale) + 1;
                row = obj.y_max*obj.scale - floor(y_pos * obj.scale);
                cell = obj.obstacle_array(row, col);
                valid = cell == 0;
                return;
            else
                valid = false;
                return;
            end
        end
        
        function [x_pos,y_pos] = gen_rand_pos(obj)
            % Generate a random postion within the bounds and free of obstacles
            
            for i = 1:100 % try up to 100 times
                x_pos = rand(1) * (obj.x_max - obj.x_min) + obj.x_min;
                y_pos = rand(1) * (obj.y_max - obj.y_min) + obj.y_min;

                if obj.check_pos(x_pos, y_pos)
                    return;
                end
                if i == 100
                    fprintf('Failed to generate random position');
                    return;
                end
            end
        end
    end
end

