clear all;
tic;
rng(1);

table_center = [0; -317.5/1000; 0];
% Hardware specs for the hand.
tip_radius = 2.5 / 1000;
% 3 spacings, each is 6mm.
width_finger = 18/ 1000;

% Use the COM (trianglular centroid) as origin of local frame.
shape_info.shape_id = 'polygon1';
shape_info.shape_type = 'polygon';
virtual_buffer = 0.001;
le_long = 58.0 / 1000.0 + virtual_buffer;
le_short = 27.5 / 1000.0 + virtual_buffer;

shape_info.shape_vertices = [le_long / 3, -le_long*2/3, le_long/3; -le_short/3, -le_short/3, le_short * 2 /3];
shape_info.pho = le_short * 2 / 3;

options_support_pts.mode = 'polygon';
options_support_pts.vertices = shape_info.shape_vertices';
num_support_pts = 50;

%support_pts = GridSupportPoint(num_support_pts, options_support_pts); % N*2.
support_pts = densifyPolygon(shape_info.shape_vertices', num_support_pts);

num_support_pts = size(support_pts, 1);
options_pressure.mode = 'uniform';
pressure_weights = AssignPressure(support_pts, options_pressure);

% limit surface fitting based on pressure distribution.
ls_type = 'quadratic';
% Uncomment the following two lines if you first run this file. 
pushobj = PushedObject(support_pts', pressure_weights, shape_info, ls_type);
pushobj.FitLS(ls_type, 50, 0.1);
pushobj.noise_df = 10000;
A = pushobj.ls_coeffs;
a = A(1,1);
b_normalized = A(3,3);
pushobj.ls_coeffs = diag([a;a;b_normalized]);
b = b_normalized / (pushobj.pho^2);

hand_two_finger = ConstructTwoRoundFingersGripperHand(tip_radius);

mu = 0.25;
q_end = table_center;

% Longer right angle edge: contact point right below COM. 
hand_local_pt_1  = [0;  -le_short/3 - tip_radius];
np_1 = [0;1];
push_action_1 = PushActionDubins(hand_local_pt_1, np_1,  mu, a, b);
% Shorter right angle edge: mid point. The point is asymmetric. 
hand_local_pt_2  = [le_long/3 + tip_radius; le_short / 6];
np_2 = [-1;0];
push_action_2 = PushActionDubins(hand_local_pt_2, np_2, mu, a, b);
% Longest edge: contact point is symmetric point. The projection of COM onto the edge. 
qx = le_long * (2/3 * le_long^2 + 1/3 * le_short^2 ) / (le_long^2 + le_short^2) - 2/3 * le_long;
qy = le_short * (2/3 * le_long^2 + 1/3 * le_short^2 ) / (le_long^2 + le_short^2) - 1/3 * le_short; 
r_extend = (norm([qx;qy]) + tip_radius) / norm([qx;qy]);
hand_local_pt_3  = [qx;qy] * r_extend;
np_3 = [-qx; - qy] / norm([qx;qy]);
push_action_3 = PushActionDubins(hand_local_pt_3, np_3, mu, a, b);

num_steps = 49;
%[traj_localframe, traj_pusherframe] = push_action.PlanDubinsPath(q_start, q_end, num_steps);

all_push_actions = {push_action_1, push_action_2, push_action_3};
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
%[traj_obj, traj_pusher, action_ids] = plan_graph.GetCompleteObjectHandPath( way_pts, action_records, 30);
%csv_file_path = '/home/jiaji/catkin_ws/src/dubins_pushing/test_multi_actions.csv';
%table_z_h = 0.325; PrintPusherCartesianTrajectoryMultiAction(traj_pusher, action_ids, table_z_h, csv_file_path);
