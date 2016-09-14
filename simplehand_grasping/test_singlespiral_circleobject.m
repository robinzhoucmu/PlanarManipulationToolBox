addpath('~/Downloads/cvx/');
cvx_startup;
shape_info.shape_id = 'circle1';
shape_info.shape_type = 'circle';
shape_info.shape_parameters.radius = 0.02;
shape_info.pho = shape_info.shape_parameters.radius;

options_support_pts.mode = 'circle';
options_support_pts.range = shape_info.shape_parameters.radius;

num_supports_pts = 200; 
support_pts = GridSupportPoint(num_supports_pts, options_support_pts); % N*2.

options_pressure.mode = 'uniform';
pressure_weights = AssignPressure(support_pts, options_pressure);

ls_coeff = [    1.004487776488424
   1.005250998296516
   5.354783646156310
   0.002398361293719
   0.000000000001541
   0.008249800764895
   0.000000000000036
   0.150706348334593
   0.000000000000066
   2.004813856921745
   7.455281550335771
   7.435194683091581
   0.000000000000073
   0.021749766452242
   0.078381957959822
];
ls_type = 'poly4';

pushobj = PushedObject(support_pts', pressure_weights, shape_info, ls_type, ls_coeff);
pushobj.FitLS('poly4', 400, 0);
%ls_coeff_quad = diag([1.1417, 1.0792, 2.4776]);
%ls_type_quad = 'quadratic';
%pushobj = PushedObject(support_pts', pressure_weights, shape_info, ls_type_quad, ls_coeff_quad);

% Set the circle at the point of origin.
 pushobj.pose = [-shape_info.shape_parameters.radius * 0.3;
     shape_info.shape_parameters.radius * 0.7;
     0];
 pushobj.pose = [-0.01138; -0.01645; 0];
%tmpangle = pi*2/3;
%tmpangle = 0;
%tmpPosition = [cos(tmpangle), - sin(tmpangle); sin(tmpangle), cos(tmpangle)] * cursor_info.Position';
%pushobj.pose = [tmpPosition;0];
const_mu = 0.1;
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



