rng(1);
tic;
%% Construct pushobj.
% shape and support points.
shape_info.shape_id = 'polygon1';
shape_info.shape_type = 'polygon';
le = 0.02;
shape_info.shape_vertices = [-le,le,le,-le;-le,-le,le,le];
shape_info.pho = le;                                            
% Uniformly sample points in the polygon area 
options_support_pts.mode = 'polygon';
options_support_pts.vertices = shape_info.shape_vertices';
num_support_pts = 20;

support_pts = GridSupportPoint(num_support_pts, options_support_pts); % N*2.
num_support_pts = size(support_pts, 1);
options_pressure.mode = 'uniform';
pressure_weights = AssignPressure(support_pts, options_pressure);
%figure, plot(support_pts(:,1), support_pts(:,2), '^');
%axis equal;
%drawnow;

% limit surface fitting based on pressure distribution.
ls_type = 'quadratic';
% Uncomment the following two lines if you first run this file. 
%pushobj = PushedObject(support_pts', pressure_weights, shape_info, ls_type);
%pushobj.FitLS(ls_type, 200, 0.1);

% put the object initially at the origin.
pushobj.pose= [le/3;0;pi/24];

virtual_circle_radius = le/20;
finger_radius = virtual_circle_radius;
finger_length = le / 2;
finger_width = finger_length / 4;
[hand_parallel_edges] = ConstructParallelEdgeGripper(virtual_circle_radius, finger_length, finger_width);

q_start = [0; 0; -pi/2; 3*sqrt(2) * le + virtual_circle_radius *2];
q_end = [-le; 0; -pi/2; 2*le + virtual_circle_radius * 2];


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

mu = 0.1;
sim_inst = ForwardSimulationCombinedStateNewGeometry(pushobj, hand_traj, hand_parallel_edges , mu);
sim_results = sim_inst.RollOut();
toc;
num_rec_configs = size(sim_results.obj_configs, 2);
figure;
hold on;
seg_size = 5;
for i = 1:1:num_rec_configs
    if mod(i, seg_size) == 1
    % Plot the square object.
        plot(sim_results.obj_configs(1, i), sim_results.obj_configs(2,i), 'b+');
        vertices = SE2Algebra.GetPointsInGlobalFrame(pushobj.shape_vertices, sim_results.obj_configs(:,i));
        vertices(:,end+1) = vertices(:,1);
        plot(vertices(1,:), vertices(2,:), 'r-');
     % Plot the round point pusher.
        theta = sim_results.hand_configs(3,i);
        d = sim_results.hand_configs(4,i);
        R = [cos(theta), -sin(theta);
                sin(theta), cos(theta)];
        vec_f1 = R * [0;d/2];
        drawCircle(sim_results.hand_configs(1,i) + vec_f1(1), sim_results.hand_configs(2,i)+ vec_f1(2), finger_radius, 'k');
        drawCircle(sim_results.hand_configs(1,i) - vec_f1(1), sim_results.hand_configs(2,i) - vec_f1(2), finger_radius, 'k');
    end
end
axis equal;
