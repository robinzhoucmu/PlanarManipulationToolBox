file_name = 'motion_surface=plywood_shape=rect1_a=0_v=10_i=0.000_s=0.000_t=0.349.json';
flag_plot = 1;
shape_id = 'rect1';
[object_pose, tip_pose, wrench] = get_and_plot_data(file_name, shape_id, flag_plot);

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
% Uncomment the following two lines if you first run this file. 
pushobj = PushedObject(support_pts', pressure_weights, shape_info, ls_type);
pushobj.FitLS(ls_type, 400, 0.1);
% Specify the initial pose of the object.
pushobj.pose = object_pose(1,2:end)';
%% Construct the hand and its trajectory.
% Construct a single round point pusher with specified radius.
hand_single_finger = ConstructSingleRoundFingerHand(tip_radius);
% Specify its trajectory.
hand_traj_opts.q = tip_pose(:,2:end)';
hand_traj_opts.t = bsxfun(@minus, tip_pose(:,1), tip_pose(1,1));
hand_traj_opts.interp_mode = 'spline';
hand_traj = HandTraj(hand_traj_opts);

%% Simulation.
mu = 0.8;
dt_collision = 0.01;
pushobj.pose = object_pose(1,2:end)';
sim_inst = ForwardSimulation(pushobj, hand_traj, hand_single_finger, mu, dt_collision);
[sim_results] = sim_inst.RollOut();

%% Plotting
num_rec_configs = size(sim_results.obj_configs, 2);
figure;
hold on;
seg_size = 20;
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

