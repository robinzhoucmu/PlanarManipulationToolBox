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


