function [record_all] = EvaluatePositionOffsetMCubeData(folder_name, surface_type, shape_id, vel, ls_type, num_samples_perfile)
tic;
rng(1);
query_info.surface= surface_type; 
query_info.shape = shape_id; 
query_info.velocity = vel; 
if (nargin < 6)
    num_samples_perfile = 20;
end
[all_wrenches_local, all_twists_local, vel_tip_local, dists, vel_slip] = read_json_files(folder_name, query_info, num_samples_perfile); 
all_twists_local_normalized = UnitNormalize(all_twists_local);
num_sample_pairs = 500;
sampledindices = datasample(1:1:size(all_wrenches_local,1), num_sample_pairs,'Replace',false);
% Fit model using sampled data.
if strcmp(ls_type, 'poly4')
    [ls_coeffs, xi, delta, pred_V, s] = Fit4thOrderPolyCVX(all_wrenches_local(sampledindices,:)', ...
        all_twists_local_normalized(sampledindices,:)', 1, 1, 1, 1);
elseif strcmp(ls_type, 'quadratic')
    [ls_coeffs, xi, delta, pred_V, s] = FitEllipsoidForceVelocityCVX(all_wrenches_local(sampledindices,:)', ...
        all_twists_local_normalized(sampledindices,:)', 1, 1, 1, 1);
else
    error('%s type not recognized\n', ls_type);
end

% Get all json file listing.
str_folder = strcat(folder_name, '/', query_info.surface, '/', query_info.shape, '/')
% example: 'motion_surface=plywood_shape=rect1_a=0_v=10_i=0.000_s=0.000_t=0.349.json'
str_file = strcat('motion_surface=', query_info.surface, '_shape=', query_info.shape, ...
                  '*v=', num2str(query_info.velocity), '_i*.json')
listing = dir(strcat(str_folder, str_file));    
% Remove all near-empty files.
for i = 1:1:length(listing)
    if listing(i).bytes < 100
        delete(strcat(str_folder,listing(i).name));
    end
end
addpath(str_folder);
listing = dir(strcat(str_folder, str_file));    
num_files = length(listing);

% Construct pushobject.
tip_radius = 0.00475;
[pho] = compute_shape_avgdist_to_center(shape_id);
shape_vertices = get_shape(shape_id);
shape_vertices(end,:) = [];

shape_info.shape_id = shape_id;
shape_info.shape_type = 'polygon';
shape_info.shape_vertices = shape_vertices';
shape_info.pho = pho;
% Create the push object with given ls_type and coefficient.
pushobj = PushedObject([], [], shape_info, ls_type, ls_coeffs);
% Construct a single round point pusher with specified radius.
hand_single_finger = ConstructSingleRoundFingerHand(tip_radius);

record_all = cell(num_files, 1);
%record_all.init_pose_gt = zeros(3, num_files);
%record_all.final_pose_gt = zeros(3, num_files);
%record_all.final_pose_sim = zeros(3, num_files);
parfor i = 1:1:num_files
    (i+0.0)/num_files
    file_name = listing(i).name
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

    mu = 0.22;
    pushobj = PushedObject([], [], shape_info, ls_type, ls_coeffs);
    % Set initial pose.
    pushobj.pose = object_pose(1,:)';
    sim_inst = ForwardSimulationCombinedState(pushobj, hand_traj, hand_single_finger, mu);
    [sim_results] = sim_inst.RollOut();

    record_all{i}.init_pose_gt = object_pose(1,:)';
    record_all{i}.final_pose_gt = object_pose(end,:)';
    record_all{i}.final_pose_sim = sim_results.obj_configs(:, end);
%     record_all.init_pose_gt(:, i) = object_pose(1,:)';
%     record_all.final_pose_gt(:, i) = object_pose(end,:)';
%     record_all.final_pose_sim(:, i) = sim_results.obj_configs(:, end);
end
toc;
end