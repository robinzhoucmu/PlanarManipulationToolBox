function [record_all, record_ls_training] = EvaluatePositionOffsetMCubeData(folder_name, surface_type, shape_id, vel, ls_type, mu, num_samples_perfile, ratio_training_files, flag_uniform_pressure)
tic;
p = gcp;
if (isempty(p))
    num_physical_cores = feature('numcores');
    parpool(num_physical_cores);
end
query_info.surface= surface_type; 
query_info.shape = shape_id; 
query_info.velocity = vel; 
if (nargin < 7)
    num_samples_perfile = 20;
end
if (nargin < 8)
    ratio_training_files = 0.5;
end
if (nargin < 9) 
    flag_uniform_pressure = 0;
end
weight_angle_to_disp = 1;
weight_wrench = 1;
weight_twist = 1;
tip_radius = 0.00475;
[pho] = compute_shape_avgdist_to_center(shape_id);
shape_vertices = get_shape(shape_id);
shape_vertices(end,:) = [];

shape_info.shape_id = shape_id;
shape_info.shape_type = 'polygon';
shape_info.shape_vertices = shape_vertices';
shape_info.pho = pho;

file_listing = GetMCubeFileListing(folder_name, query_info);
index_file_training = rand(length(file_listing), 1) < ratio_training_files;
file_listing_training = file_listing(index_file_training);
file_listing_testing = file_listing(~index_file_training);
% for debugging. Let's only use 10% of the testing files. 
rand_perm = randperm(length(file_listing_testing));
file_listing_testing = file_listing_testing(rand_perm(1:ceil(length(file_listing_testing))));
num_training_data = 600;
if (~flag_uniform_pressure)
    [all_wrenches_local, all_twists_local, vel_tip_local, dists, vel_slip] = read_json_files(file_listing_training, query_info.shape, num_samples_perfile); 
    all_twists_local_normalized = UnitNormalize(all_twists_local);
    num_sample_pairs = min(num_training_data, size(all_wrenches_local, 1))
    sampledindices = datasample(1:1:size(all_wrenches_local,1), num_sample_pairs,'Replace',false);
    all_wrenches_local_training = all_wrenches_local(sampledindices,:);
    all_twists_local_training = all_twists_local(sampledindices,:);
    all_twists_local_normalized_training = all_twists_local_normalized(sampledindices,:);
    % Fit model using sampled data.
    if strcmp(ls_type, 'poly4')
        [ls_coeffs, xi, delta, pred_V, s] = Fit4thOrderPolyCVX(all_wrenches_local_training', ...
            all_twists_local_normalized_training', weight_twist, weight_wrench, 1, 1);
    elseif strcmp(ls_type, 'quadratic')
        [ls_coeffs, xi, delta, pred_V, s] = FitEllipsoidForceVelocityCVX(all_wrenches_local_training', ...
            all_twists_local_normalized_training', weight_twist, weight_wrench, 1, 1);
    else
        error('%s type not recognized\n', ls_type);
    end
    record_ls_training.wrenches = all_wrenches_local_training;
    record_ls_training.twists = all_twists_local_training;
else
    options_support_pts.mode = 'polygon';
    options_support_pts.vertices = shape_info.shape_vertices';
    num_supports_pts = 50;
    support_pts = GridSupportPoint(num_supports_pts, options_support_pts); % N*2.
    %num_supports_pts = size(support_pts, 1);
    options_pressure.mode = 'uniform';
    pressure_weights = AssignPressure(support_pts, options_pressure);
    result_ls_training.support_pts = support_pts;
    result_ls_training.pressure_weights = pressure_weights;
end

num_files = length(file_listing_testing);

% Construct pushobject
if (~flag_uniform_pressure)
% Create the push object with given ls_type and coefficient.
    pushobj = PushedObject([], [], shape_info, ls_type, ls_coeffs);
else
    pushobj = PushedObject(support_pts', pressure_weights, shape_info, ls_type);
    pushobj.FitLS(ls_type, 150, 0.1);    
end
ls_coeffs = pushobj.ls_coeffs;
record_ls_training.ls_coeffs = pushobj.ls_coeffs;
record_ls_training.ls_type = pushobj.ls_type;
% Construct a single round point pusher with specified radius.
hand_single_finger = ConstructSingleRoundFingerHand(tip_radius);
% Grid search over mu to find the best value on training data.
mu_trials = [mu-0.075;mu-0.05;mu-0.025;mu;mu+0.025;mu+0.05;mu+0.075];
%mu_trials = [mu];
mu_best = 0;
val_best = 1e+3;
ct_mu = 1;
while ct_mu <= length(mu_trials)
    all_dev = zeros(length(file_listing_training), 1);
parfor i = 1:1:length(file_listing_training)
    file_name = file_listing_training(i).name;
    [object_pose, tip_pose, wrench] = get_and_plot_data(file_name, query_info.shape, 0);
    N = floor((tip_pose(end,1) - tip_pose(1,1)) / 0.01);
    [object_pose, tip_pt, force, t_q] = interp_data(object_pose, tip_pose, wrench, N);
    tip_pose = [tip_pt,zeros(length(t_q), 1)];
    
    % Specify its trajectory.
    hand_traj_opts = [];
    hand_traj_opts.q = tip_pose';
    hand_traj_opts.t = bsxfun(@minus, t_q, t_q(1));
    hand_traj_opts.interp_mode = 'spline';
    hand_traj = HandTraj(hand_traj_opts);

    pushobj = PushedObject([], [], shape_info, ls_type, ls_coeffs);
    % Set initial pose.
    pushobj.pose = object_pose(1,:)';
    sim_inst = ForwardSimulationCombinedState(pushobj, hand_traj, hand_single_finger, mu_trials(ct_mu));
    [sim_results] = sim_inst.RollOut();
    alpha = mod(sim_results.obj_configs(3,end) + 10 * pi, 2*pi);
    beta = mod(object_pose(end,3) + 10 *pi, 2*pi);
    all_dev(i) =  norm(sim_results.obj_configs(1:2,end) - object_pose(end,1:2)') + ...
       weight_angle_to_disp * pushobj.pho * abs(compute_angle_diff(alpha, beta))
end
if (sum(all_dev) < val_best)
    val_best = sum(all_dev);
    mu_best = mu_trials(ct_mu);
end
ct_mu = ct_mu + 1;
end
mu_best
val_best / length(file_listing_training)
record_ls_training.mu_best = mu_best;

% Record testing result.
record_all = cell(num_files, 1);
parfor i = 1:1:num_files
    file_name = file_listing_testing(i).name;
    [object_pose, tip_pose, wrench] = get_and_plot_data(file_name, query_info.shape, 0);
    N = floor((tip_pose(end,1) - tip_pose(1,1)) / 0.01);
    [object_pose, tip_pt, force, t_q] = interp_data(object_pose, tip_pose, wrench, N);
    tip_pose = [tip_pt,zeros(length(t_q), 1)];
    
    % Specify its trajectory.
    hand_traj_opts = [];
    hand_traj_opts.q = tip_pose';
    hand_traj_opts.t = bsxfun(@minus, t_q, t_q(1));
    hand_traj_opts.interp_mode = 'spline';
    hand_traj = HandTraj(hand_traj_opts);

    pushobj = PushedObject([], [], shape_info, ls_type, ls_coeffs);
    % Set initial pose.
    pushobj.pose = object_pose(1,:)';
    sim_inst = ForwardSimulationCombinedState(pushobj, hand_traj, hand_single_finger, mu_best);
    [sim_results] = sim_inst.RollOut();

    record_all{i}.init_pose_gt = object_pose(1,:)';
    record_all{i}.final_pose_gt = object_pose(end,:)';
    record_all{i}.final_pose_sim = sim_results.obj_configs(:, end);
    record_all{i}.init_tip_pt = tip_pt(1,:)';
    record_all{i}.final_tip_pt = tip_pt(end, :)';
    record_all{i}.file_name = file_name;
%     record_all.init_pose_gt(:, i) = object_pose(1,:)';
%     record_all.final_pose_gt(:, i) = object_pose(end,:)';
%     record_all.final_pose_sim(:, i) = sim_results.obj_configs(:, end);
end
toc;
end