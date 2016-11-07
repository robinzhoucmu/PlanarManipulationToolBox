rng(1);
tic;
%% Construct pushobj.
% shape and support points.
% shape_info.shape_id = 'tri1';
% shape_info.shape_type = 'polygon';
 le = 0.02;
% % right, upper, left. ccw.
% shape_info.shape_vertices = [le/2,0,-le/2;-le/sqrt(3)/2, le/sqrt(3), -le/sqrt(3)/2];
% shape_info.pho = le / sqrt(3);                                            
% % Uniformly sample points in the polygon area 
% options_support_pts.mode = 'polygon';
% options_support_pts.vertices = shape_info.shape_vertices';
% num_support_pts = 100;
% 
% support_pts = GridSupportPoint(num_support_pts, options_support_pts); % N*2.
% num_support_pts = size(support_pts, 1);
% options_pressure.mode = 'uniform';
% pressure_weights = AssignPressure(support_pts, options_pressure);
% % figure, plot(support_pts(:,1), support_pts(:,2), '^');
% % axis equal;
% % drawnow;
% 
% % limit surface fitting based on pressure distribution.
% ls_type = 'quadratic';
% Uncomment the following two lines if you first run this file. 
%pushobj = PushedObject(support_pts', pressure_weights, shape_info, ls_type);
%pushobj.FitLS(ls_type, 300, 0.1);
pushobj = CreateNSidedPolygonPushObject(3, le, 'quadratic');
% put the object initially at the origin.
pushobj.pose= [le/4;le/4;pi/6];

%% Construct hand.
finger_radius = 0.002;
hand_three_finger = ConstructThreeFingersOneDofHand(finger_radius);
%% Specify hand trajectory.
% Way points
q_start = [0; 0; pi/3;  0.75 * le + finger_radius];
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
hand_traj = HandTraj(hand_traj_opts);

%% Set simulation.
mu = 0.1;
dt_collision = 0.05;

sim_inst = ForwardSimulationCombinedState(pushobj, hand_traj, hand_three_finger, mu);
sim_results = sim_inst.RollOut();

num_rec_configs = size(sim_results.obj_configs, 2);
h = figure;
hold on;
seg_size = 10;
for i = 1:1:num_rec_configs
    if mod(i, seg_size) == 1
    % Plot the square object.
        plot(sim_results.obj_configs(1, i), sim_results.obj_configs(2,i), 'b+');
        vertices = SE2Algebra.GetPointsInGlobalFrame(pushobj.shape_vertices, sim_results.obj_configs(:,i));
        vertices(:,end+1) = vertices(:,1);
        plot(vertices(1,:), vertices(2,:), 'r-');
        if (i == 1) || (i + seg_size > num_rec_configs)
            c = 'k';
        else
            c = 'g';
        end
        hand_three_finger.Draw(h, sim_results.hand_configs(:, i), c);
    end
end
axis equal;
toc;