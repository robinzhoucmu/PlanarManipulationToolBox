% This function constructs a N-sided polygon with specified edge length and limit surface type. 
% It samples 100 points uniformly inside the polygonal area and then
% samples (around) 300 wrench-twist pairs to fit the limit surface specified by
% ls_type.
function [pushobj_polygon, shape_info] = CreateNSidedPolygonPushObject(num_edges, len_edge, ls_type)
    shape_info.shape_vertices = zeros(2, num_edges);
    shape_info.shape_type = 'polygon';
    shape_info.shape_id = strcat('polygon', num2str(num_edges));
    alpha = 2*pi / num_edges;
    r = len_edge * sqrt(1.0/ (2*(1 - cos(alpha))));
    shape_info.pho = r;
    for i = 1:1:num_edges
        theta = alpha * (i-1) + pi/2;
        shape_info.shape_vertices(:, i) = r * [cos(theta); sin(theta)];
    end
    opt_support_pts.mode = 'polygon';
    opt_support_pts.vertices = shape_info.shape_vertices';
    num_support_pts = 100;
    support_pts = GridSupportPoint(num_support_pts, opt_support_pts); % N*2.
    
    options_pressure.mode = 'uniform';
    pressure_weights = AssignPressure(support_pts, options_pressure);
    
    num_wrench_twist_pairs = 50;
    ratio_facet = 0.1;
    pushobj_polygon = PushedObject(support_pts', pressure_weights, shape_info, ls_type);
    tic;
    display('Start to fit limit surface parameteric model.');
    pushobj_polygon.FitLS(ls_type, num_wrench_twist_pairs, ratio_facet);
    display('Complete the fitting procedure.')
    toc;
end