clear all;
tic;
rng(1);

table_center = [0; -317.5/1000; 0];
% Hardware specs for the hand.
tip_radius = 2.5 / 1000;
% 3 spacings, each is 6mm.
width_finger = 18/ 1000;


shape_id = 'butter';
shape_info.shape_id = shape_id;
shape_info.shape_type = 'polygon';
[rho] = compute_shape_avgdist_to_center(shape_id) * 0.3;
shape_vertices = get_shape(shape_id);
shape_vertices(end,:) = [];
shape_vertices = shape_vertices';
shape_vertices(1,:) = shape_vertices(1,:) * 0.3;
shape_vertices(2,:) = shape_vertices(2,:) * 0.25;
shape_info.shape_vertices = shape_vertices;

support_pts = shape_info.shape_vertices';
%------------------------------------

num_support_pts = size(support_pts, 1);
options_pressure.mode = 'uniform';
pressure_weights = AssignPressure(support_pts, options_pressure);

shape_info.pho = rho;                                            


% limit surface fitting based on pressure distribution.
ls_type = 'quadratic';
% Uncomment the following two lines if you first run this file. 
pushobj = PushedObject(support_pts', pressure_weights, shape_info, ls_type);
pushobj.FitLS(ls_type, 50, 0.1);
pushobj.noise_df = 1000;
pushobj.nsides_symmetry = 2;
x =shape_info.shape_vertices(1,:); y = shape_info.shape_vertices(2,:); k = convhull(x,y);
xmax = max(x(k));
ymax = max(y(k));

A = pushobj.ls_coeffs;
a = A(1,1);
b_normalized = A(3,3);
pushobj.ls_coeffs = diag([a;a;b_normalized]);
b = b_normalized / (pushobj.pho^2);

hand_two_finger = ConstructTwoRoundFingersGripperHand(tip_radius);

mu = 0.2;
%q_start = pose_vision_node;
q_end = table_center;


hand_local_pt_1  = [-xmax - 2*tip_radius +  0.00314; 0];
np_1 = [1;0];
push_action_1 = PushActionDubins(hand_local_pt_1, np_1,  mu, a, b);

hand_local_pt_2  = [ xmax + 2*tip_radius -  0.00314; 0];
np_2 = [-1;0];
push_action_2 = PushActionDubins(hand_local_pt_2, np_2, mu, a, b);

hand_local_pt_3  = [ 0; - ymax - 2*tip_radius + 0.0044;];
np_3 = [0;1];
push_action_3 = PushActionDubins(hand_local_pt_3, np_3, mu, a, b);

hand_local_pt_4  = [ 0; ymax + 2*tip_radius - 0.0044;];
np_4 = [0;-1];
push_action_4 = PushActionDubins(hand_local_pt_4, np_4, mu, a, b);

num_steps = 49;

all_push_actions = {push_action_1, push_action_2, push_action_3, push_action_4};
pose_goal = q_end;
table_size_x = (451.6 - 30) / 1000.0;
table_size_y = (254.0 - 30)/ 1000.0;
range_pose_min = [q_end(1) - table_size_x / 2; q_end(2) - table_size_y / 2; -pi];
range_pose_max = [q_end(1) + table_size_x / 2; q_end(2) + table_size_y / 2; pi ];
nd = 10;
cost_switch = 0.01;
plan_graph = PlanningGraph(all_push_actions, pose_goal);
plan_graph.SetRangeAndDiscretization(range_pose_min, range_pose_max, nd);
tic;
plan_graph.ConstructGraph(cost_switch);
toc;

 [way_pts, action_records, min_path_length] = plan_graph.QueryNewStartPose( [0; -317.5/1000; pi/2])
 plan_graph.VisualizePlannedPath(pushobj, hand_two_finger, way_pts, action_records);
