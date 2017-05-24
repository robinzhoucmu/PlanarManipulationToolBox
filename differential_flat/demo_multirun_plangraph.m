clear all;
close all;

%load exp_data/plan_graph_triangle_mu03nd15.mat;
%load exp_data/plan_graph_bigrect_mu03nd12_cs03.mat;
%load exp_data/plan_graph_butter_mu03nd12_cs05.mat;
load exp_data/plan_graph_butter_mu025nd10_cs05_vbhalfmm.mat;

%dq = [0,0,pi/2; 30,50, pi/4; -50, 20, pi;];   % 3 cases for red triangle.
%dq = [20,-30,pi/2; 30,50, pi/3; 60, 20, pi*5/6 ;];  % 3 cases for big rect. 
dq = [-30,10,pi/2; 20,-10, pi/6; 60, 20, pi*3/4 ;]; % 3 cases for butter.
dq(:,1:2) = dq(:,1:2) / 1000.0;
for i = 1:1:size(dq, 1)
  q_start = [0 + dq(i,1); -317.5/1000 + dq(i,2); 0 + dq(i,3)]
 [way_pts, action_records, min_path_length] = plan_graph.QueryNewStartPose(q_start)
 plan_graph.VisualizePlannedPath(pushobj, hand_two_finger, way_pts, action_records);
 [traj_obj, traj_pusher, action_ids] = plan_graph.GetCompleteObjectHandPath( way_pts, action_records, 30);
csv_file_path = '~/catkin_ws/src/dubins_pushing/test_multi_actions.csv';
table_z_h = 0.323;
PrintPusherCartesianTrajectoryMultiAction(traj_pusher, action_ids, table_z_h, csv_file_path);
 %waitforbuttonpress;
 input('next');
 close all;
end
 
 

%csv_read_file = '~/catkin_ws/src/dubins_pushing/scripts/output.txt';
%VisualizePushingExpLog(csv_read_file, pushobj, hand_two_finger, 18/1000)
%ImproveFigure(gcf)