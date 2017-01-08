% Caveat: Penetration case does not work: sometimes return empty
% or missing collision pairs.
% Input: 
% poly_a can be just a simple point. However, poly b must be a proper
% polygon.
% poly_a, poly_b: (2*Na, 2*Nb) polygon a and b geometry in their respective local frame.
% pose_a, pose_b: the pose in world frame. 
% Output: unique pairs of minimum distance contact pairs. 
% 4*N column vectors, first 2 elements in each column
% corresponds to vertices of A, last 2 elements are their respective
% projection on polygon B
function [closest_pairs, min_dist] = PolygonToPolygonContactInfo(poly_a, poly_b, pose_a, pose_b)
% Get the vertices coordinates in world frame.
 [cur_vertices_poly_a] = GetPolygonShapeInWorldFrame(poly_a, pose_a);
 [cur_vertices_poly_b] = GetPolygonShapeInWorldFrame(poly_b, pose_b);
 
 % If polygon A is not a degenerate point.
 if size(cur_vertices_poly_a, 2) > 1
    % Compute the minimum distance between the polygons.
    min_dist = distancePolygons(cur_vertices_poly_a', cur_vertices_poly_b');
    % Important to get rid of chattering between two contact points on an
    % edge.
    r_scaled = 1 + 1e-2;
    min_dist_scaled = min_dist * r_scaled;
    % Project vertices of A onto polygon B.
    [pts_pairs_va_polyb, indices_va_polyb_closest] = ProjectPointsOntoPolygonDistanceThreshold(...
     cur_vertices_poly_a, cur_vertices_poly_b, min_dist_scaled);
    % Project vertices of B onto polygon A.
    [pts_pairs_vb_polya, indices_vb_polya_closest] = ProjectPointsOntoPolygonDistanceThreshold(...
         cur_vertices_poly_b, cur_vertices_poly_a, min_dist_scaled);
     % Swap to <a,b> order
     pts_pairs_vb_polya = [pts_pairs_vb_polya(3:4, :); pts_pairs_vb_polya(1:2, :)];
     closest_pairs = [pts_pairs_va_polyb'; pts_pairs_vb_polya'];
     closest_pairs = unique(closest_pairs, 'rows');
     closest_pairs = closest_pairs';
 else
    [closest_pairs, min_dist] = ProjectPointOntoPolygon(cur_vertices_poly_a, cur_vertices_poly_b);
 end
end




%% Test codes.
% case 1: edge to vertex
% h = 0; b = -1; poly1 = [10 h ; 10 20 ; b 20 ; b h; 10 h;];
% pose1 = [0;0;pi/4];
% [closest_pairs] = PolygonToPolygonContactInfo(poly1', poly2', pose1, [0;0;0]); figure; hold on; 
% drawPolyline(GetPolygonShapeInWorldFrame(poly1', pose1)', 'b');
% drawPolyline(poly2, 'm'); axis equal; closest_pairs

% case 2: edge to edge
% h = 2; b = 0; poly1 = [10 h ; 10 20 ; b 20 ; b h; 10 h;];
% pose1 = [0;0;0];
% [closest_pairs,dist] = PolygonToPolygonContactInfo(poly1', poly2', pose1, [0;0;0]); figure; hold on; 
% drawPolyline(GetPolygonShapeInWorldFrame(poly1', pose1)', 'b');
% drawPolyline(poly2, 'm'); axis equal; closest_pairs,dist

% case 3: point to polygon.
% poly1 = [2 0;];
% pose1 = [0;0;0];
% [closest_pairs, dist] = PolygonToPolygonContactInfo(poly1', poly2', pose1, [0;0;0]); figure; hold on; 
% plot(poly1(1), poly1(2), 'r*');
% drawPolyline(poly2, 'm'); axis equal; closest_pairs, dist

