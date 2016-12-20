clear all;
flag_plot = 0;
rng(1);
p = gcp;
if (isempty(p))
    num_physical_cores = feature('numcores');
    parpool(num_physical_cores);
end
le = 0.02;
mu = 0.2;
%% Specify hand trajectory.
% Way points
%% Construct hand.
finger_radius = 0.002;
q_start = [0; 0; 0;  2.25 * le + finger_radius];
q_end = [0; 0; 0; le * sqrt(3)/2 + finger_radius];

num_way_q = 100;
dim_q = length(q_start);
waypoints_hand_q = zeros(dim_q, num_way_q);
for i = 1:1:dim_q
    waypoints_hand_q(i,:) = linspace(q_start(i), q_end(i), num_way_q);
end
t_max = 4;
t_q = linspace(0, t_max, num_way_q);
hand_traj_opts.q = waypoints_hand_q;
hand_traj_opts.t = t_q;
hand_traj_opts.interp_mode = 'spline';
ls_type = 'poly4';
num_sides = 6;
[pushobj_hex,shape_info] = CreateNSidedPolygonPushObject(num_sides, le, ls_type);

tic;
num_poses_xy = 100;
num_poses_theta = 11; 
num_poses = num_poses_xy * num_poses_theta;
sample_radius = le;
sample_angle = pi * 2.0 / 6;
sampled_ic_poses = cylindricalsampling(sample_radius, sample_angle, num_poses_xy, num_poses_theta);
sd = num_poses^(1/3);
sim_results_all = cell(num_poses, 1);
parfor ind_pose = 1:1:num_poses
    pushobj = PushedObject(pushobj_hex.support_pts, pushobj_hex.pressure_weights, ...
        shape_info, pushobj_hex.ls_type, pushobj_hex.ls_coeffs);
    pushobj.pose = sampled_ic_poses(:, ind_pose);
    hand_three_finger = ConstructThreeFingersOneDofHand(finger_radius);
    hand_traj = HandTraj(hand_traj_opts);
    sim_inst = ForwardSimulationCombinedState(pushobj, hand_traj, hand_three_finger, mu);
    sim_results_all{ind_pose} = sim_inst.RollOut();
    ind_pose
end
toc;
q_inits = zeros(3, num_poses);
q_ends = zeros(3, num_poses);
for ind_pose = 1:1:num_poses
        q_inits(:, ind_pose) = sim_results_all{ind_pose}.obj_configs(:,1);
        q_ends(:, ind_pose) = sim_results_all{ind_pose}.obj_configs(:,end);
end
if (flag_plot)
    [h1, h2] = PlotPrePostDistributions(sim_results_all, pushobj_hex.pho,0, 0);
    [h3] = PlotObjectConfigurationTrajectory(sim_results_all, pushobj_hex.pho,0,0);
%     figure; seg = 10; 
%     ks = 4;
%     for i = (ks)*num_poses_xy:1:(ks+1)*num_poses_xy
%         traj_obj = sim_results_all{i}.obj_configs; traj_obj(1:2,:) = traj_obj(1:2, :) / pushobj_tri.pho;
%         quiver3(traj_obj(1,1:seg:end-1), traj_obj(2,1:seg:end-1), traj_obj(3,1:seg:end-1), traj_obj(1,2:seg:end) - traj_obj(1,1:seg:end-1), traj_obj(2,2:seg:end) - traj_obj(2,1:seg:end-1), traj_obj(3,2:seg:end) -traj_obj(3,1:seg:end-1) , 'MaxHeadSize', 0.2);
%         hold on;
%     end
end
str_datetime = datestr(datetime('now'));
str_file_to_save = strcat('data_logs/', str_datetime);
save(str_file_to_save, 'sim_results_all', 'q_inits', 'q_ends', 'pushobj_hex');
% hand_for_plot = ConstructThreeFingersOneDofHand(finger_radius);
% for ind_pose = 1:1:num_poses
%     sim_results = sim_results_all{ind_pose};
%     num_rec_configs = size(sim_results.obj_configs, 2);
%     h = figure;
%     hold on;
%     seg_size = 10;
%     for i = 1:1:num_rec_configs
%         if mod(i, seg_size) == 1
%             if (i == 1) || (i + seg_size > num_rec_configs)
%                 c = 'k';
%                 c_obj = 'b';
%             else
%                 c = 'g';
%                 c_obj = 'r';
%             end
%         % Plot the square object.
%             plot(sim_results.obj_configs(1, i), sim_results.obj_configs(2,i), 'b+');
%             vertices = SE2Algebra.GetPointsInGlobalFrame(pushobj_tri.shape_vertices, sim_results.obj_configs(:,i));
%             vertices(:,end+1) = vertices(:,1);
%             plot(vertices(1,:), vertices(2,:), '-', 'Color', c_obj);
%             hand_for_plot.Draw(h, sim_results.hand_configs(:, i), c);
%         end
%     end
%     axis equal;    
% end
