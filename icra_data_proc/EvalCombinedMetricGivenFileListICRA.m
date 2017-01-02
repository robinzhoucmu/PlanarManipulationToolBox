function [avg_combined_metric, record_all] = EvalCombinedMetricGivenFileListICRA(record_log, file_indices, ls_type, ls_coeffs, shape_info, hand_single_finger, mu)
devs = zeros(length(file_indices), 1);
weight_angle_to_disp = 1;
record_all = cell(length(file_indices), 1);

parfor i = 1:1:length(file_indices) 
    ind_trial = file_indices(i);
    % Specify finger trajectory.
    hand_poses = record_log.robot_2d_pos_full{ind_trial}';
    hand_traj_opts = [];
    hand_traj_opts.q = hand_poses;
    hand_traj_opts.t = linspace(0,1, size(hand_poses, 2));
    hand_traj_opts.interp_mode = 'spline';
    hand_traj = HandTraj(hand_traj_opts);

    pushobj = PushedObject([], [], shape_info, ls_type, ls_coeffs);
    object_poses = record_log.obj_2d_traj{ind_trial}';
    pushobj.pose = object_poses(:,1);
    sim_inst = ForwardSimulationCombinedState(pushobj, hand_traj, hand_single_finger, mu);
    [sim_results] = sim_inst.RollOut();
    alpha = mod(sim_results.obj_configs(3,end) + 10 * pi, 2*pi);
    beta = mod(object_poses(3, end) + 10 *pi, 2*pi);
    devs(i) =  norm(sim_results.obj_configs(1:2,end) - object_poses(1:2, end)) + ...
          weight_angle_to_disp * pushobj.pho * abs(compute_angle_diff(alpha, beta));
   
    record_all{i}.init_pose_gt = object_poses(:,1);
    record_all{i}.final_pose_gt = object_poses(:,end);
    record_all{i}.final_pose_sim = sim_results.obj_configs(:, end); 
end
avg_combined_metric = mean(devs);
end