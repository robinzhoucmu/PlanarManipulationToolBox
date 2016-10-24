rng(1);
%% Construct pushobj.
% shape and support points.
shape_info.shape_id = 'polygon1';
shape_info.shape_type = 'polygon';
le = 0.02;
shape_info.shape_vertices = [-le,le,le,-le;
                                                -le,-le,le,le];
                         
options_support_pts.mode = 'polygon';
options_support_pts.vertices = shape_info.shape_vertices';
num_support_pts = 20;

support_pts = GridSupportPoint(num_supports_pts, options_support_pts); % N*2.
num_supports_pts = size(support_pts, 1);
options_pressure.mode = 'uniform';
pressure_weights = AssignPressure(support_pts, options_pressure);
%figure, plot(support_pts(:,1), support_pts(:,2), '^');
%axis equal;
%drawnow;

% limit surface fitting based on pressure distribution.
ls_type = 'quadratic';
%pushobj = PushedObject(support_pts', pressure_weights, shape_info, ls_type);
%pushobj.FitLS(ls_type, 400, 0.1);

% put the object initially at the origin.

%% Construct hand.

fun_fk_hand_single_finger = @(q)([0;0;0]); 
fun_fv_hand_single_finger = @(q, qdot)([0;0;0]);

hand_single_finger = Hand();
hand_single_finger.num_fingers = 1;
hand_single_finger.finger_radius = 0.005;
hand_single_finger.fun_fk = fun_fk_hand_single_finger;
hand_single_finger.fun_fv = fun_fv_hand_single_finger;

%% Specify hand trajectory.
% Way points
q_start = [0; -le * 2; 0];
q_end = [le; le*4; 0];
num_way_q = 20;
dim_q = length(q_start);
waypoints_hand_q = zeros(dim_q, num_way_q);
for i = 1:1:dim_q
    waypoints_hand_q(i,:) = linspace(q_start(i), q_end(i), num_way_q);
end
t_max = 4;
t_q = linspace(0, t_max, num_way_q);
hand_traj_opts.q = waypoints_hand_q;
hand_traj_opts.t = t_q;
hand_traj_opts.interp_mode = 'spline';
hand_traj = HandTraj(hand_traj_opts);

%% Set simulation.
mu = 0.5;
dt_collision = 0.05;