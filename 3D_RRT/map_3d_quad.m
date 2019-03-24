classdef map_3d_quad
    %MAP_3D_QUAD Quads in a half-edge mesh used to represent map data
    % This could also be used for any n-gon, where n > 2
    
    properties
        certainty   % Measure of how well explored the quad is. Ranges from 0 to 1
        variance    % Measure of variance of points sampled to create quad. Ranges from 0 upward 
    end
    
    methods
        function obj = map_3d_quad(certainty, variance)
            %MAP_3D_QUAD Construct an instance of this class
            obj.certainty = certainty;
            obj.variance = variance;
        end
    end
end

