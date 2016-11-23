file_name_1 = 'motion_surface=plywood_shape=rect1_a=0_v=10_i=0.000_s=0.000_t=0.349.json';
file_name_2 = 'motion_surface=plywood_shape=rect1_a=0_v=10_i=1.000_s=0.300_t=0.000.json';
file_name_3 = 'motion_surface=plywood_shape=ellip1_a=0_v=10_i=0.000_s=0.025_t=-0.698.json';
file_name_test = 'motion_surface=delrin_shape=ellip1_a=0_v=10_i=0.000_s=0.525_t=0.698.json';
flag_plot = 1;
%shape_id = 'rect1';
shape_id = 'ellip1';
addpath(strcat('~/pushing_data/', 'delrin/', shape_id, '/'));
[object_pose, tip_pose, wrench] = get_and_plot_data(file_name_test, shape_id, flag_plot);
N = floor((tip_pose(end,1) - tip_pose(1,1)) / 0.01);
[object_pose, tip_pt, force, t_q] = interp_data(object_pose, tip_pose, wrench, N);
tip_pose = [tip_pt,zeros(length(t_q), 1)];

tip_radius = 0.00475;
shape_vertices = get_shape(shape_id);
shape_vertices(end,:) = [];
[pho] = compute_shape_avgdist_to_center(shape_id);

%% Construct the push object.
shape_info.shape_id = shape_id;
shape_info.shape_type = 'polygon';
shape_info.shape_vertices = shape_vertices';
shape_info.pho = pho;
% Uniformly sample points in the polygon area 
% and assign uniform pressure.  

options_support_pts.mode = 'polygon';
options_support_pts.vertices = shape_info.shape_vertices';

%options_support_pts.mode = 'rim';
%options_support_pts.range = pho;

num_supports_pts = 50;
support_pts = GridSupportPoint(num_supports_pts, options_support_pts); % N*2.
num_supports_pts = size(support_pts, 1);
options_pressure.mode = 'uniform';
pressure_weights = AssignPressure(support_pts, options_pressure);
%figure, plot(support_pts(:,1), support_pts(:,2), '^');
%axis equal;
%drawnow;
% limit surface fitting based on pressure distribution.
ls_type = 'poly4';
ls_coeff_test = [0.146370788404678
   0.113197964421117
   0.278638393573682
   0.058360895230491
   0.026172121870120
  -0.039808858375788
   0.052015077985983
  -0.120560048277458
   0.073685879884696
   0.177299808465202
   0.620482600921318
   1.024614815650334
   0.016548222766705
   0.029731679316744
  -0.278834553794510];
pushobj = PushedObject([],[],shape_info,ls_type, ls_coeff_test);
% Uncomment the following two lines if you first run this file. 
%pushobj = PushedObject(support_pts', pressure_weights, shape_info, ls_type);
%pushobj.FitLS(ls_type, 150, 0.1);
% Specify the initial pose of the object.
pushobj.pose = object_pose(1,2:end)';
%% Construct the hand and its trajectory.
% Construct a single round point pusher with specified radius.
hand_single_finger = ConstructSingleRoundFingerHand(tip_radius);
% Specify its trajectory.
hand_traj_opts.q = tip_pose';
hand_traj_opts.t = bsxfun(@minus, t_q, t_q(1));
hand_traj_opts.interp_mode = 'spline';
hand_traj = HandTraj(hand_traj_opts);

%% Simulation.
tic;
mu = 0.15;
pushobj.pose = object_pose(1,:)';
%sim_inst = ForwardSimulation(pushobj, hand_traj, hand_single_finger, mu, dt_collision);
sim_inst = ForwardSimulationCombinedState(pushobj, hand_traj, hand_single_finger, mu);
[sim_results] = sim_inst.RollOut();
toc;
%% Plotting
num_rec_configs = size(sim_results.obj_configs, 2);
figure;
hold on;
seg_size = 10;
for i = 1:1:num_rec_configs
    if mod(i, seg_size) == 1
    % Plot the square object.
        plot(sim_results.obj_configs(1, i), sim_results.obj_configs(2,i), 'b+');
        vertices = SE2Algebra.GetPointsInGlobalFrame(pushobj.shape_vertices, sim_results.obj_configs(:,i));
        vertices(:,end+1) = vertices(:,1);
        plot(vertices(1,:), vertices(2,:), 'r-');
     % Plot the round point pusher.
        drawCircle(sim_results.hand_configs(1,i), sim_results.hand_configs(2,i), hand_single_finger.finger_radius, 'k');
    end
end
axis equal;

