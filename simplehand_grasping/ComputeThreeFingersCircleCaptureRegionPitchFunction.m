function [all_results] = ComputeThreeFingersCircleCaptureRegionPitchFunction(pushobj, pitch_fun, mu, ratio_uncertainty, finger_radius, num_init_samples)
rng(1);
uncertainty_radius = pushobj.shape_parameters.radius * ratio_uncertainty;

max_quasistatic_vel = pushobj.shape_parameters.radius; % one radius per second.
finger_movement_sample_dt = 0.01;

eps_init_dist = 0.001;
finger_traj = PitchCompute(pitch_fun, ... 
                           max_quasistatic_vel, ...
                           pushobj.shape_parameters.radius, ...
                           uncertainty_radius + finger_radius + eps_init_dist, ...
                           finger_movement_sample_dt);

finger_traj.plot();
simulation_inst = SimulationWorld(pushobj, finger_traj, mu, finger_radius, 3);
ct_samples = 0;
all_results = cell(num_init_samples, 1);
while (ct_samples < num_init_samples)
    (ct_samples + 0.0) / num_init_samples
    % Sample a random initial pose such that the object circle center is
    % within (ratio_uncertainty - 1) * circle_radius.
    sampling_radius = (uncertainty_radius - pushobj.shape_parameters.radius);
    init_pose = 2 * bsxfun(@minus, rand(2,1), 0.5) * sampling_radius;
    % Rejection sampling.
    if norm(init_pose) < sampling_radius
        ct_samples = ct_samples + 1;
        % Set initial pose of the object.
        simulation_inst.pushobj.pose = [init_pose;0];
        [flags, pose_log] = simulation_inst.SimulationRollOut(0);
        all_results{ct_samples}.init_pose = init_pose;
        all_results{ct_samples}.result_flags = flags;
        all_results{ct_samples}.pose_log = pose_log;
    end
end

% Plot capture regions from result logs.
figure;
hold on;
num_grasped = 0;
num_jammed = 0;
num_missed = 0;
for i = 1:1:ct_samples
    if (all_results{i}.result_flags.grasped == 1)
        plot(all_results{i}.init_pose(1), all_results{i}.init_pose(2), 'r+');
        num_grasped = num_grasped + 1;
    elseif (all_results{i}.result_flags.jammed == 1)
        plot(all_results{i}.init_pose(1), all_results{i}.init_pose(2), 'k*');
        num_jammed = num_jammed + 1;
    else
        plot(all_results{i}.init_pose(1), all_results{i}.init_pose(2), 'bo');
        num_missed = num_missed + 1;
    end
end
end