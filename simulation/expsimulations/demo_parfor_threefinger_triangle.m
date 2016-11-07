sim_results_all = cell(3, 1);
p = gcp;
if (isempty(p))
    parpool(2);
end
le = 0.02;
mu = 0.1;
%% Specify hand trajectory.
% Way points
%% Construct hand.
finger_radius = 0.002;
q_start = [0; 0; pi/3;  1.25 * le + finger_radius];
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
%[pushobj_tri,shape_info] = CreateNSidedPolygonPushObject(3, le, 'quadratic');

tic;
parfor ind_pose = 1:1:3
    pushobj = PushedObject(pushobj_tri.support_pts, pushobj_tri.pressure_weights, ...
        shape_info, pushobj_tri.ls_type, pushobj_tri.ls_coeffs);
    pushobj.pose = [ind_pose* le/10; ind_pose*le/10; ind_pose*pi/6];
    hand_three_finger = ConstructThreeFingersOneDofHand(finger_radius);
    hand_traj = HandTraj(hand_traj_opts);
    sim_inst = ForwardSimulationCombinedState(pushobj, hand_traj, hand_three_finger, mu);
    sim_results_all{ind_pose} = sim_inst.RollOut();
end
hand_for_plot = ConstructThreeFingersOneDofHand(finger_radius);
for ind_pose = 1:1:3
    sim_results = sim_results_all{ind_pose};
    num_rec_configs = size(sim_results.obj_configs, 2);
    h = figure;
    hold on;
    seg_size = 10;
    for i = 1:1:num_rec_configs
        if mod(i, seg_size) == 1
            if (i == 1) || (i + seg_size > num_rec_configs)
                c = 'k';
                c_obj = 'b';
            else
                c = 'g';
                c_obj = 'r';
            end
        % Plot the square object.
            plot(sim_results.obj_configs(1, i), sim_results.obj_configs(2,i), 'b+');
            vertices = SE2Algebra.GetPointsInGlobalFrame(pushobj_tri.shape_vertices, sim_results.obj_configs(:,i));
            vertices(:,end+1) = vertices(:,1);
            plot(vertices(1,:), vertices(2,:), '-', 'Color', c_obj);
            hand_for_plot.Draw(h, sim_results.hand_configs(:, i), c);
        end
    end
    axis equal;    
end
toc;