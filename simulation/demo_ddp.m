rng(1);
shape_info.shape_id = 'polygon1';
shape_info.shape_type = 'polygon';
le = 1;
shape_info.shape_vertices = [-le,le,le,-le;
                                                -le,-le,le,le];
shape_info.pho = le;                                            
% Uniformly sample points in the polygon area 
% and 
options_support_pts.mode = 'polygon';
options_support_pts.vertices = shape_info.shape_vertices';
num_support_pts = 25;

support_pts = GridSupportPoint(num_support_pts, options_support_pts); % N*2.
num_support_pts = size(support_pts, 1);
options_pressure.mode = 'uniform';
pressure_weights = AssignPressure(support_pts, options_pressure);
%figure, plot(support_pts(:,1), support_pts(:,2), '^');
%axis equal;
%drawnow;

% limit surface fitting based on pressure distribution.
ls_type = 'poly4';
% Uncomment the following two lines if you first run this file. 
pushobj = PushedObject(support_pts', pressure_weights, shape_info, ls_type);
pushobj.FitLS(ls_type, 200, 0.1);

% put the object initially at the origin.
pushobj.pose= [0;0;0];

%% Construct hand.
finger_radius = 0.1;
hand_single_finger = ConstructSingleRoundFingerHand(finger_radius);
% Set initial pose of the hand.
hand_single_finger.q = [0;-1 - hand_single_finger.finger_radius;0];

mu = 1;
N = 200;
T = 0.01;
total_T = N * T;

x0 = [0 0 0]';
xstar = [-2 6 -pi/6]';
pt = [0; -1];
%A  = eye(3,3);
%A = diag([1, 1.1 , 2]);
A = pushobj.ls_coeffs;
opts_dynamics.A = A;
% Corrupt the A matrix by a bit.
scale = 0.1;
opts_dynamics.A = A +  scale * diag(max(rand(3,1),0));
opts_dynamics.A
A
% Compute friction cone.
normal = [0;-1];
f_c = ComputeFrictionConeEdges(pt, normal, mu, pushobj.pho);
[ vc_edges ] = ComputeVelConeGivenFC(f_c, opts_dynamics.A, pushobj.ls_type);
% Compute point velocity cone.
B = [1, 0, -pt(2)/pushobj.pho;
     0, 1, pt(1)/pushobj.pho];
vp_cone_edges = B * vc_edges;

opts_dynamics.N = N;
opts_dynamics.dt = T;
opts_dynamics.pt =  pt;
opts_dynamics.motioncone = vp_cone_edges;

% state cost 
Q = 200*[20 2]'; % running cost, final cost
ep = 5 * [0.1 0.1 0.05]'; % Huber Loss
R = 20*diag([0.5 0.5]);
opts_costs.Q = Q;
opts_costs.ep = ep;
opts_costs.R = R;

ddp_controller = DDPController(opts_dynamics, opts_costs);
ddp_controller.SetInitialPose(x0);
ddp_controller.SetGoalPose(xstar);
ddp_controller.TrainController();

% add some noise in initial condition.
sn = 0.1;
pushobj.pose = x0 + sn * [1;1;0.5]; 
perturb_mu = 0;
mu = mu + perturb_mu;
sim_inst = ForwardSimulationCombinedStateCloseLoop(pushobj, ddp_controller, hand_single_finger, mu);
[sim_results] = sim_inst.RollOut(N * T, T/2);

% plotting. 
num_rec_configs = size(sim_results.obj_configs, 2);
figure;
hold on;
% plot the goal in blue.
vertices = SE2Algebra.GetPointsInGlobalFrame(pushobj.shape_vertices, xstar);
vertices(:,end+1) = vertices(:,1);
plot(vertices(1,:), vertices(2,:), 'k-');
% plot the initial state (for training) in green.
vertices = SE2Algebra.GetPointsInGlobalFrame(pushobj.shape_vertices, ddp_controller.xinit);
vertices(:,end+1) = vertices(:,1);
plot(vertices(1,:), vertices(2,:), 'g-');

seg_size = 15;
for i = 1:1:num_rec_configs
    if mod(i, seg_size) == 1
    % Plot the square object.
        plot(sim_results.obj_configs(1, i), sim_results.obj_configs(2,i), 'r+');
        vertices = SE2Algebra.GetPointsInGlobalFrame(pushobj.shape_vertices, sim_results.obj_configs(:,i));
        vertices(:,end+1) = vertices(:,1);
        plot(vertices(1,:), vertices(2,:), 'r-');
     % Plot the round point pusher.
        drawCircle(sim_results.hand_configs(1,i), sim_results.hand_configs(2,i), hand_single_finger.finger_radius, 'k');
    end
end
axis equal;
sim_results.obj_configs(:,end)