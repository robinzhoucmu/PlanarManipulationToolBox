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
    min_dist = distancePolygons(cur_vertices_poly_a', cur_vertices_poly_b')
    % Project vertices of A onto polygon B.
    [pts_pairs_va_polyb, indices_va_polyb_closest] = ProjectPointsOntoPolygonDistanceThreshold(...
     cur_vertices_poly_a, cur_vertices_poly_b, min_dist);
    % Project vertices of B onto polygon A.
    [pts_pairs_vb_polya, indices_vb_polya_closest] = ProjectPointsOntoPolygonDistanceThreshold(...
         cur_vertices_poly_b, cur_vertices_poly_a, min_dist);
     % Swap to <a,b> order
     pts_pairs_vb_polya = [pts_pairs_vb_polya(3:4, :); pts_pairs_vb_polya(1:2, :)];
     closest_pairs = [pts_pairs_va_polyb'; pts_pairs_vb_polya'];
     closest_pairs = unique(closest_pairs, 'rows');
     closest_pairs = closest_pairs';
 else
    [closest_pairs, min_dist] = ProjectPointOntoPolygon(cur_vertices_poly_a, cur_vertices_poly_b);
 end
end

% Project the vertices of polygon A onto polygon B and store the pairs that
% are smaller than a distance threshold.
% Output: pts_pairs: 4*N column vectors, first 2 elements in each column
% corresponds to vertices of A, last 2 elements are their respective
% projection on polygon B. 
function [pts_pairs, indices_va_polyb_closest] = ProjectPointsOntoPolygonDistanceThreshold(cur_vertices_poly_a, cur_vertices_poly_b, min_dist)
    % If the polygon A is NOT a degenerate point.
    if (size(cur_vertices_poly_a, 2) > 1)
        % Compute the closest points on poly b w.r.t vertices of a. 
         [dists_va_polyb, locations_polyb] = distancePointPolyline(cur_vertices_poly_a', cur_vertices_poly_b');
         % Find pairs that are of the minimum distance.
         indices_va_polyb_closest = (dists_va_polyb <= min_dist);
         % Log these pairs. 
         pts_polyb_project = (polygonPoint(cur_vertices_poly_b', locations_polyb))';
         pts_polyb_closest = pts_polyb_project(:, indices_va_polyb_closest);
         pts_polya_closest = cur_vertices_poly_a(:, indices_va_polyb_closest);
         pts_pairs = [pts_polya_closest;pts_polyb_closest];
    else
    end
end
% Input: pt 2*1 vector. polygon 2*N.
% Output: pts_pair [pt; pt_proj] and minimum distance.
function [pts_pair, dist] = ProjectPointOntoPolygon(pt, polygon)
    [location, dist] = projPointOnPolygon(pt', polygon');
    pts_pair = [pt; polygonPoint(polygon', location)'];
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

