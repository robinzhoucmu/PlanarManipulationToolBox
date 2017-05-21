rng(1);
table_center = [0; -317.5/1000; 0];
% Hardware specs for the hand.
tip_radius = 2.5 / 1000;
% 4 spacings, each is 6mm.
width_finger = 24/ 1000;

shape_info.shape_id = 'polygon1';
shape_info.shape_type = 'polygon';
extra_len = 0.001;
le_short = 0.035 + extra_len;
le_long = 0.05 + extra_len;
shape_info.shape_vertices = 0.5 * [-le_long, le_long, le_long, -le_long;-le_short, -le_short, le_short, le_short];
le = (le_short + le_long)/4;
shape_info.pho = le;

options_support_pts.mode = 'polygon';
options_support_pts.vertices = shape_info.shape_vertices';
num_support_pts = 20;

support_pts = GridSupportPoint(num_support_pts, options_support_pts); % N*2.
num_support_pts = size(support_pts, 1);
options_pressure.mode = 'uniform';
pressure_weights = AssignPressure(support_pts, options_pressure);

% limit surface fitting based on pressure distribution.
ls_type = 'quadratic';
% Uncomment the following two lines if you first run this file. 
%pushobj = PushedObject(support_pts', pressure_weights, shape_info, ls_type);
%pushobj.FitLS(ls_type, 50, 0.1);
pushobj.noise_df = 10000;
A = pushobj.ls_coeffs;
a = A(1,1);
b_normalized = A(3,3);
pushobj.ls_coeffs = diag([a;a;b_normalized]);
b = b_normalized / (pushobj.pho^2);

pose_vision_node = [ -35.0830485244/1000; ...
   -378.8867871/1000;  ...
     -0.0297944656226];
%pose_start = pose_vision_node - table_center;
hand_two_finger = ConstructTwoRoundFingersGripperHand(tip_radius);

mu = 1.5;
q_start = pose_vision_node;
q_end = table_center;

hand_local_pt_1  = [-le_long/2 - tip_radius; 0];
np_1 = [1;0];
push_action_1 = PushActionDubins(hand_local_pt_1, np_1, 0.5 * mu, a, b);

hand_local_pt_2  = [le_long/2 + tip_radius; 0];
np_2 = [-1;0];
push_action_2 = PushActionDubins(hand_local_pt_2, np_2, 0.5 * mu, a, b);

hand_local_pt_3  = [ 0; -le_short/2 - tip_radius;];
np_3 = [0;1];
push_action_3 = PushActionDubins(hand_local_pt_3, np_3, 0.5 * mu, a, b);

num_steps = 49;
%[traj_localframe, traj_pusherframe] = push_action.PlanDubinsPath(q_start, q_end, num_steps);

all_push_actions = {push_action_1, push_action_2, push_action_3};
pose_goal = q_end;
table_size_x = 451.6 / 1000.0;
table_size_y = 254.0 / 1000.0;
range_pose_min = [q_end(1) - table_size_x / 2; q_end(2) - table_size_y / 2; -pi];
range_pose_max = [q_end(1) + table_size_x / 2; q_end(2) + table_size_y / 2; pi - 0.02];
nd = 8;
cost_switch = 0.01;
plan_graph = PlanningGraph(all_push_actions, pose_goal);
plan_graph.SetRangeAndDiscretization(range_pose_min, range_pose_max, nd);
tic;
plan_graph.ConstructGraph(cost_switch);
toc;

hand_two_finger = ConstructTwoRoundFingersGripperHand(tip_radius);
[way_pts, action_records, tot_path_length] = plan_graph.GetShortestPath(11);
plan_graph.VisualizePlannedPath(pushobj, hand_two_finger, way_pts, action_records);


