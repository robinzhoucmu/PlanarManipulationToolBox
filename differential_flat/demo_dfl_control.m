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

%zeta0 = 0.01;
zeta0 = 0.05;
k_scale = 1;
kv1 =  k_scale * 2;
kv2 =  k_scale * 1;
kv = [kv1, kv2];
kp = [0.99*(kv1.^2/4) , 0.99*(kv2.^2/4)];
freq = 500;
dfl_controller = PostureControllerDFL(freq, kp, kv, zeta0);
dfl_controller.SetSystemParameters(a, b, r, mu);

%q_start =  [-6*le; -5.0*le; pi/2];
%q_start =  [6*le; 0; 0];
%q_start =  [-le; 3*le; pi];
q_start =  [-0.2*le; -0.2*le; pi/8];
%q_start =  [-0.1*le; -2*le; -pi/12];
pushobj.pose = q_start;
hand_single_finger.q = [q_start(1); q_start(2); 0] - r*[-sin(q_start(3));cos(q_start(3));0];


sim_inst = ForwardSimulationCombinedStateNewGeometryWithController(pushobj, dfl_controller, hand_single_finger, mu+0.2);
t_max =10.0;
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