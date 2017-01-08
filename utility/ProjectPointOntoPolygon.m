% Input: pt 2*1 vector. polygon 2*N.
% Output: pts_pair [pt; pt_proj] and minimum distance.
function [pts_pair, dist] = ProjectPointOntoPolygon(pt, polygon)
    [location, dist] = projPointOnPolygon(pt', polygon');
    pts_pair = [pt; polygonPoint(polygon', location)'];
end