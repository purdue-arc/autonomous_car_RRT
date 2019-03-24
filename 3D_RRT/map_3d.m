classdef map_3d
    %MAP_3D This class stores data for representing a 3d map of drivable surfaces
    %   It uses a mesh of quads to store map data. Quads are explicity modeled and contain no geometry data, half-edges are modeled, while points are not
    
    properties
        current_edge% Current edge used while traversing map
        num_quads   % Number of quads in the map
        num_points  % Number of points in the map
    end
    
    methods
        function obj = map(inputArg1,inputArg2)
            %MAP Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg1 + inputArg2;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
        
        function add_quad(points)
        end
        
        function patch = generate_patch(obj)
            % GENERATE_PATCH This method generates a patch object from the map
            % It crawls through the map creating a table of point and faces
            % I expect it will be pretty expensive
            
            % Output Data
            point_matrix = zeros(obj.num_points, 3);    % Matrix of points. Each point has X,Y,Z
            quad_matrix = zeros(obj.num_quads, 4);      % Matrix of quads. Each quad has 4 points
            
            % Temporary Data
            quad_list(obj.num_quads) = map_3d_quad(0,0);    % Create a list to track the quads that we have been to
            quad_list_index = 1;                            % Index into the list
            edge_queue(obj.num_quads) = obj.current_edge;   % Create a queue to track the edges that we need to visit
            edge_queue_remove_index = obj.num_quads;        % Matlab doesn't have queue's so instead we have a vector and index. Close enough for now
            edge_queue_add_index = obj.num_quads - 1;       % Index to add to
            
            % Algorithim
            while quad_list_index <= obj.num_quads          % Stop after we have filled in the last quad
                current_edge = edge_queue_index(edge_queue_remove_index);
                % Add next queued edge's quad to list
                quad_list(quad_list_index) = current_edge.parent_quad;
                edge_queue_remove_index = edge_queue_remove_index - 1;
                quad_list_index = quad_list_index + 1;
                for edge_num = 1:4                          % For each of the 4 edges around a quad
                    if(current_edge.twin_edge ~= 0)
                        % Add twin to queue
                    
            
            
            
            
            end
            
            
            
            
            
            
            
            
            
            
            
    end
end

