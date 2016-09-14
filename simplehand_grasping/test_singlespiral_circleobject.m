addpath('~/Downloads/cvx/');
cvx_startup;
shape_info.shape_id = 'circle1';
shape_info.shape_type = 'circle';
shape_info.shape_parameters.radius = 0.02;
shape_info.pho = shape_info.shape_parameters.radius;

options_support_pts.mode = 'circle';
options_support_pts.range = shape_info.shape_parameters.radius;

num_supports_pts = 100; 
support_pts = GridSupportPoint(num_supports_pts, options_support_pts); % N*2.

options_pressure.mode = 'uniform';
pressure_weights = AssignPressure(support_pts, options_pressure);

ls_coeff = [ 1.0276 
    1.0381
    5.1589
   -0.0300
    0.4683
   -0.0498
   -0.4392
    0.5807
   -0.5780
    2.0527
    7.3867
    7.3668
   -0.3526
    0.3345
   -0.1457
];
ls_type = 'poly4';

pushobj = PushedObject(support_pts', pressure_weights, shape_info, ls_type, ls_coeff);
%pushobj.FitLS('quadratic', 200, 0.2);
%ls_coeff_quad = diag([1.1417, 1.0792, 2.4776]);
%ls_type_quad = 'quadratic';
%pushobj = PushedObject(support_pts', pressure_weights, shape_info, ls_type_quad, ls_coeff_quad);

% Set the circle at the point of origin.
 pushobj.pose = [-shape_info.shape_parameters.radius * 0.3;
     shape_info.shape_parameters.radius * 0.7;
     0];
 pushobj.pose = [0.01; 0.01433; 0];
%tmpangle = pi*2/3;
%tmpangle = 0;
%tmpPosition = [cos(tmpangle), - sin(tmpangle); sin(tmpangle), cos(tmpangle)] * cursor_info.Position';
%pushobj.pose = [tmpPosition;0];
const_mu = 0.05;
finger_radius = 0.002;

max_quasistatic_vel = shape_info.shape_parameters.radius; % one radius per second.

finger_movement_sample_dt = 0.01;
eps_init_dist = 0.001;
pitch = 0;
finger_traj = PitchCompute(@(t)(pitch), max_quasistatic_vel, ...
    shape_info.shape_parameters.radius, 2 * shape_info.shape_parameters.radius + finger_radius + eps_init_dist, finger_movement_sample_dt);

simulation_case = SimulationWorld(pushobj, finger_traj, const_mu, finger_radius, 3);
flag_plot = 1;
flag_stop_at_first_contact = 1;
[flags, pose_log, twists, center_linear_vel] = simulation_case.SimulationRollOut(flag_plot, flag_stop_at_first_contact);
flags
twists
center_linear_vel



