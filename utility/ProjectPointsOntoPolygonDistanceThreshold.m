% Project the vertices of polygon A onto polygon B and store the pairs that
% are smaller than a distance threshold.
% Output: pts_pairs: 4*N column vectors, first 2 elements in each column
% corresponds to vertices of A, last 2 elements are their respective
% projection on polygon B. 
function [pts_pairs, indices_va_polyb_closest] = ProjectPointsOntoPolygonDistanceThreshold(cur_vertices_poly_a, cur_vertices_poly_b, min_dist)
    % If the polygon A is NOT a degenerate point.
    if (size(cur_vertices_poly_a, 2) > 1)
        % Compute the closest points on poly b w.r.t vertices of a. 
        % Need the following if. Follow the geo2d function.
        if sum(cur_vertices_poly_b(:, end) == cur_vertices_poly_b(:, 1)) ~= 2
            cur_vertices_poly_b = [cur_vertices_poly_b, cur_vertices_poly_b(:, 1)];
        end
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