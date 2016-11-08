clear all;
rng(1);
p = gcp;
if (isempty(p))
    num_physical_cores = feature('numcores');
    parpool(num_physical_cores);
end
le = 0.02;
mu = 0.1;
%% Specify hand trajectory.
% Way points
%% Construct hand.
finger_radius = 0.002;
q_start = [0; 0; pi/3;  (2/sqrt(3) + 0.1) * le + finger_radius];
q_end = [0; 0; pi/3; le/4 + finger_radius];

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
[pushobj_tri,shape_info] = CreateNSidedPolygonPushObject(3, le, 'quadratic');

tic;
num_poses = 8^3;
sd = num_poses^(1/3);
sim_results_all = cell(num_poses, 1);
parfor ind_pose = 1:1:num_poses
    pushobj = PushedObject(pushobj_tri.support_pts, pushobj_tri.pressure_weights, ...
        shape_info, pushobj_tri.ls_type, pushobj_tri.ls_coeffs);
    % Sample a pose uniformly.
    %pushobj.pose = [le/sqrt(3) * 2 * (rand() - 0.5); le/sqrt(3) * 2  * (rand() - 0.5);  pi/3 * 2 * (rand() -0.5)];
    k1 = floor(ind_pose / sd^2);
    k2 = floor((ind_pose - k1*sd^2) / sd);
    k3 = floor(ind_pose - k1*sd^2 - k2*sd);
    pushobj.pose = [le/sqrt(3) * 2*(k1 / sd - 0.5); le/sqrt(3) * 2*(k2 / sd - 0.5);  pi/3 * 2*(k3 / sd - 0.5)];
    hand_three_finger = ConstructThreeFingersOneDofHand(finger_radius);
    hand_traj = HandTraj(hand_traj_opts);
    sim_inst = ForwardSimulationCombinedState(pushobj, hand_traj, hand_three_finger, mu);
    sim_results_all{ind_pose} = sim_inst.RollOut();
end
toc;

h1 = figure; 
h2 = figure;
hold on;
for ind_pose = 1:1:num_poses
    q_init = sim_results_all{ind_pose}.obj_configs(:,1);
    q_end = sim_results_all{ind_pose}.obj_configs(:,end);
    figure(h1);
    plot3(q_init(1)/pushobj_tri.pho, q_init(2)/pushobj_tri.pho, q_init(3), 'b*');
    hold on;
    figure(h2);
    plot3(q_end(1)/pushobj_tri.pho, q_end(2)/pushobj_tri.pho, q_end(3), 'r*');
    hold on;
end

figure; seg = 10; 
for i = 1:1:num_poses
traj_obj = sim_results_all{i}.obj_configs; traj_obj(1:2,:) = traj_obj(1:2, :) / pushobj_tri.pho;
quiver3(traj_obj(1,1:seg:end-1), traj_obj(2,1:seg:end-1), traj_obj(3,1:seg:end-1), traj_obj(1,2:seg:end) - traj_obj(1,1:seg:end-1), traj_obj(2,2:seg:end) - traj_obj(2,1:seg:end-1), traj_obj(3,2:seg:end) -traj_obj(3,1:seg:end-1) , 'MaxHeadSize', 0.2);hold on;
end;

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
