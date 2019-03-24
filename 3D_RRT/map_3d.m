classdef map_3d
    %MAP_3D This class stores data for representing a 3d map of drivable surfaces
    %   It uses a mesh of quads to store map data. Quads are explicity modeled and contain no geometry data, half-edges are modeled, while points are not
    
    properties
        Property1
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
    end
end

