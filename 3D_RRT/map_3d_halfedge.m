classdef map_3d_halfedge < handle
    %MAP_3D_HALFEDGE Half edge data used to store map data
    
    properties
        base_vertex     % The base vector of this half edge. One of the four corners of the quad
        prev_edge       % The previous half-edge for the parent quad
        twin_edge       % The 'twin' of this half-edge. Goes in opposite direction and is child of neighboring quad.
        next_edge       % The next half-edge for the parent quad
        parent_quad     % The quad that this half edge belongs to (on left side)
    end
    
    methods
        function obj = map_3d_halfedge(base_vertex, prev_edge, twin_edge, next_edge, parent_quad)
            %MAP_3D_HALFEDGE Construct an instance of this class
            %   Standard constructor
            obj.base_vertex = base_vertex;
            obj.prev_edge = prev_edge;
            obj.twin_edge = twin_edge;
            obj.next_edge = next_edge;
            obj.parent_quad = parent_quad;
        end
        
        function normal = get_parent_normal(obj)
            %METHOD1 Computes the normal of the parent quad
            %   Uses cross product of prev X this
            this_vector = obj.next_edge.base_vertex - obj.base_vertex;
            prev_vector = obj.base_vertex - obj.prev_edge.base_vertex;
            normal = cross(prev_vector, this_vector);
        end
        
        %function intersect = get
    end
end

