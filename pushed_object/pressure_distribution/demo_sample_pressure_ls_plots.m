addpath('~/Downloads/cvx/');
cvx_startup;
shape_info.shape_id = 'circle1';
shape_info.shape_type = 'circle';
shape_info.shape_parameters.radius = 0.02;
shape_info.pho = shape_info.shape_parameters.radius;

options_support_pts.mode = 'rim';
options_support_pts.range = shape_info.shape_parameters.radius;

num_supports_pts = 100; 
support_pts = GridSupportPoint(num_supports_pts, options_support_pts); % N*2.

options_pressure.mode = 'uniform';
pressure_weights = AssignPressure(support_pts, options_pressure);

pushobj_quad = PushedObject(support_pts', pressure_weights, shape_info)
pushobj_quad.FitLS('quadratic', 200, 0.1);
pushobj_poly4 = PushedObject(support_pts', pressure_weights, shape_info)
pushobj_poly4.FitLS('poly4', 200, 0.1);


shape_info2.shape_id = 'hex';
shape_info2.shape_type = 'polygon';
shape_info2.shape_vertices = get_shape('hex');
shape_info2.shape_vertices = shape_info2.shape_vertices(1:end-1, :);
shape_info2.pho = compute_shape_avgdist_to_center('hex');

support_pts2 = shape_info2.shape_vertices * 0.75;
pressure_weights2 = ones(6,1) / 6.0;
pushobj_hex_quad = PushedObject(support_pts2', pressure_weights2, shape_info2)
pushobj_hex_quad.FitLS('quadratic', 200, 0.2);
pushobj_hex_poly4 = PushedObject(support_pts2', pressure_weights2, shape_info2)
pushobj_hex_poly4.FitLS('poly4', 200, 0.2);
