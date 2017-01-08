% Input: 
% poly_a can be just a simple point. However, poly b must be a proper
% polygon.
% poly_a, poly_b: (2*Na, 2*Nb) polygon a and b geometry in their respective local frame.
% pose_a, pose_b: the pose in world frame. 

function [min_dist] = PolygonToPolygonDistance(poly_a, poly_b, pose_a, pose_b)
% Get the vertices coordinates in world frame.
 [cur_vertices_poly_a] = GetPolygonShapeInWorldFrame(poly_a, pose_a);
 [cur_vertices_poly_b] = GetPolygonShapeInWorldFrame(poly_b, pose_b);
if size(cur_vertices_poly_a, 2) > 1
     min_dist = distancePolygons(cur_vertices_poly_a', cur_vertices_poly_b')
else
    [~,min_dist] = projPointOnPolygon(cur_vertices_poly_a', cur_vertices_poly_b');
end
end

