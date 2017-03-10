rng(1);
tic;
%% Construct pushobj.
% shape and support points.
shape_info.shape_id = 'polygon1';
shape_info.shape_type = 'polygon';
le = 0.02;
% CCW.
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
pushobj.noise_df = 10000;
A = pushobj.ls_coeffs;
a = A(1,1);
b_normalized = A(3,3);
pushobj.ls_coeffs = diag([a;a;b_normalized]);
b = b_normalized / (pushobj.pho^2);

tip_radius = le / 30;
hand_single_finger = ConstructSingleRoundFingerHand(tip_radius);


r = le + tip_radius;
mu = 1.0;

%q_start =  [2.0*le; -4.0*le; 0];
%q_start =  [6*le; 2*le; -pi/2];
%q_start =  [0; -4*le; -pi];
%q_start =  [0; 3*le; pi];
q_start = [3*le; 0;0];
%q_start =  [-0.1*le; -1*le; -pi/12];
%q_start =  [-0.1*le; -3*le; -pi/12];
%q_start =  [-0.1*le; -2*le; -pi/12];

perturbation = 0.25 * (2*rand(3,1) - ones(3,1)).* [0.01;0.01;pi/12];
pushobj.pose = q_start + perturbation;
hand_single_finger.q = [pushobj.pose(1); pushobj.pose(2); 0] - r*[-sin(pushobj.pose(3));cos(pushobj.pose(3));0];
parameters.a = a;
parameters.b = b;
parameters.r = r;
parameters.mu = mu * 0.5;
q_end = [0;0;0];
 hand_local_pt  = [0; -r];
[x, y, theta, u, z] = GetDubinPath(q_start, q_end, parameters);
% Generate trajectory.
t_max = 10.0;
interp_method = 'spline';
traj = GenerateDubinTrajectoryFromPath(z(1:2,:), t_max, interp_method);

%zeta0 = 0.01;
zeta0 = 1.0 * (u(2,1) / (t_max/length(x)) / a)
k_scale = 2.0;
% kv1 =  k_scale *1;
% kv2 =  k_scale *2;
% kv = [kv1, kv2];
% kp = [0.99*(kv1.^2/4) , 0.99*(kv2.^2/4)];
kv = k_scale * [0.2, 0.1];
kp = k_scale * [4 , 1];
freq = 30;
tracking_controller = TrackingControllerDFL(freq, kp, kv, zeta0);
tracking_controller.SetSystemParameters(a, b, r, mu*0.9);
tracking_controller.SetTrackingTrajectory(traj);

num_rec_configs = length(x);
figure;
hold on;
seg_size = 20;
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
         hold on;
         plot(hand_pt(1), hand_pt(2), 'k-');
    end
end
axis equal;
drawnow;

sim_inst = ForwardSimulationCombinedStateNewGeometryWithController(pushobj, tracking_controller, hand_single_finger, mu);
sim_results = sim_inst.RollOut(t_max);
num_rec_configs = size(sim_results.obj_configs, 2);
figure;
hold on;
seg_size = ceil(num_rec_configs / 50);
for i = 1:1:num_rec_configs
    if mod(i, seg_size) == 1
    % Plot the object.
        plot(sim_results.obj_configs(1, i), sim_results.obj_configs(2,i), 'b+');
        vertices = SE2Algebra.GetPointsInGlobalFrame(pushobj.shape_vertices, sim_results.obj_configs(:,i));
        vertices(:,end+1) = vertices(:,1);
        plot(vertices(1,:), vertices(2,:), 'r-');
     % Plot the round point pusher.
        drawCircle(sim_results.hand_configs(1,i), sim_results.hand_configs(2,i), hand_single_finger.finger_radius, 'k');
    end
end
axis equal;
sim_results.obj_configs(:,end)