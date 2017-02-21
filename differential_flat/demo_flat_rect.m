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
pushobj.noise_df = 100000;
A = pushobj.ls_coeffs;
a = A(1,1);
b_normalized = A(3,3);
pushobj.ls_coeffs = diag([a;a;b_normalized]);
b = b_normalized / (pushobj.pho^2);


tip_radius = le / 10;
hand_single_finger = ConstructSingleRoundFingerHand(tip_radius);
r = le + tip_radius;

% Get flat space dubin's car trajectory.
pose_start = [0;0;0];
pose_end = [0;-le;0];
mu = 0.3;
radius_turn =  (a / (b* r * mu));
step_size = radius_turn / 100; 
% Convert start and end cartesian pose to flat space z;
z_start(1:2) = pose_start(1:2) + a/(b*r)* [-sin(pose_start(3)); cos(pose_start(3))];
z_start(3) = pi/2;
z_end(1:2) = pose_end(1:2) + a/(b*r)* [-sin(pose_end(3)); cos(pose_end(3))];
z_end(3) = pi/2;
% Path is in world frame.
path = dubins(z_start', z_end', radius_turn, step_size);
figure, scatter(path(1,:), path(2,:));
hold on; plot(path(1,1), path(2,1), 'r*', 'MarkerSize', 10);
% Assume the entire path takes 1 second. 
T = 1.0;
num_pts = size(path, 2);
dt = T / num_pts;
[x,y,theta] = GetOrigStateFromFlatOutput(dt, path(1:2,:), a, b, r);
% Simulate open loop trajectory.
pushobj.pose= pose_start;

num_rec_configs = length(x);
hand_init_q =[0;-r;0];
hand_local_pt = [0;-r];

figure;
hold on;
seg_size = 10;
for i = 1:1:num_rec_configs
    if mod(i, seg_size) == 1 || i == num_rec_configs
    % Plot the object.
        plot(x(i), y(i), 'b+');
        obj_pose = [x(i);y(i);theta(i)];
        vertices = SE2Algebra.GetPointsInGlobalFrame(pushobj.shape_vertices, obj_pose);
        vertices(:,end+1) = vertices(:,1);
        if i == 1
            c= 'k';
        elseif i == num_rec_configs
            c = 'b';
        else
            c = 'r';
        end
        plot(vertices(1,:), vertices(2,:), '-', 'Color', c);
         R = [cos(theta(i)), -sin(theta(i)); sin(theta(i)), cos(theta(i))];
         hand_pt= R * hand_local_pt + [x(i);y(i)];
         drawCircle(hand_pt(1), hand_pt(2), hand_single_finger.finger_radius, 'k');
    end
end
axis equal;

hand_q_traj = zeros(3, num_rec_configs);
hand_q_traj(3,:) = theta;
for i = 1:1:num_rec_configs
    R = [cos(theta(i)), -sin(theta(i)); sin(theta(i)), cos(theta(i))];
    hand_q_traj(1:2, i) = R * hand_local_pt + [x(i);y(i)];
end
t_q = linspace(0, T, num_rec_configs);
hand_traj_opts.q = hand_q_traj;
hand_traj_opts.t = t_q;
hand_traj_opts.interp_mode = 'spline';
hand_traj = HandTraj(hand_traj_opts);

sim_inst = ForwardSimulationCombinedStateNewGeometry(pushobj, hand_traj, hand_single_finger, mu+0.1);
sim_results = sim_inst.RollOut();
num_rec_configs = size(sim_results.obj_configs, 2);
figure;
hold on;
seg_size = 2;
for i = 1:1:num_rec_configs
    %if mod(i, seg_size) == 1
    % Plot the object.
        plot(sim_results.obj_configs(1, i), sim_results.obj_configs(2,i), 'b+');
        vertices = SE2Algebra.GetPointsInGlobalFrame(pushobj.shape_vertices, sim_results.obj_configs(:,i));
        vertices(:,end+1) = vertices(:,1);
        plot(vertices(1,:), vertices(2,:), 'r-');
     % Plot the round point pusher.
        drawCircle(sim_results.hand_configs(1,i), sim_results.hand_configs(2,i), hand_single_finger.finger_radius, 'k');
    %end
end
axis equal;