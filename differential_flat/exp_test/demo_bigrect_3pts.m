rng(1);
table_center = [0; -317.5/1000; 0];
% Hardware specs for the hand.
tip_radius = 2.5 / 1000;
% 4 spacings, each is 6mm.
width_finger = 24/ 1000;

shape_info.shape_id = 'polygon1';
shape_info.shape_type = 'polygon';
extra_len = 0.001;
le_short = 0.035 + extra_len;
le_long = 0.05 + extra_len;
shape_info.shape_vertices = 0.5 * [-le_long, le_long, le_long, -le_long;-le_short, -le_short, le_short, le_short];
le = (le_short + le_long)/4;
shape_info.pho = le;

options_support_pts.mode = 'polygon';
options_support_pts.vertices = shape_info.shape_vertices';
num_support_pts = 20;

support_pts = GridSupportPoint(num_support_pts, options_support_pts); % N*2.
num_support_pts = size(support_pts, 1);
options_pressure.mode = 'uniform';
pressure_weights = AssignPressure(support_pts, options_pressure);

% limit surface fitting based on pressure distribution.
ls_type = 'quadratic';
% Uncomment the following two lines if you first run this file. 
%pushobj = PushedObject(support_pts', pressure_weights, shape_info, ls_type);
%pushobj.FitLS(ls_type, 50, 0.1);
pushobj.noise_df = 10000;
A = pushobj.ls_coeffs;
a = A(1,1);
b_normalized = A(3,3);
pushobj.ls_coeffs = diag([a;a;b_normalized]);
b = b_normalized / (pushobj.pho^2);

pose_vision_node = [ -35.0830485244/1000; ...
   -378.8867871/1000;  ...
     -0.0297944656226];
%pose_start = pose_vision_node - table_center;
hand_two_finger = ConstructTwoRoundFingersGripperHand(tip_radius);

mu = 1.5;
q_start = pose_vision_node;
q_end = table_center;

hand_local_pt  = [-le_long/2 - tip_radius; 0];
np = [1;0];
push_action = PushActionDubins(hand_local_pt, np, 0.5 * mu, a, b);
num_steps = 49;
[traj_localframe, traj_pusherframe] = push_action.PlanDubinsPath(q_start, q_end, num_steps);
x = traj_localframe(1,:);
y = traj_localframe(2,:);
theta = traj_localframe(3,:);

num_rec_configs = length(x)
figure;
hold on;
seg_size = 2;
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
         % hand frame point. center of the two fingers.

         hand_two_finger.q = zeros(4,1);
         hand_two_finger.q(1:3) = traj_pusherframe(:,i);
         hand_two_finger.q(3) = hand_two_finger.q(3) + pi/2.0;
         hand_two_finger.q(4) = width_finger;
         
         hold on;
         hand_two_finger.Draw(gcf);
    end
end
axis equal;
drawnow;


perturbation = 0.0 * (2*rand(3,1) - ones(3,1)).* [0.01;0.01;pi/12];
pushobj.pose = q_start + perturbation;
T = 1.0;
hand_q_traj = zeros(4, num_rec_configs);
hand_q_traj(4,:) = ones([1 num_rec_configs]) * width_finger;

hand_q_traj(1:3,:) = traj_pusherframe;
hand_q_traj(3,:) = bsxfun(@plus, pi/2, hand_q_traj(3,:));
t_q = linspace(0, T, num_rec_configs);
hand_traj_opts.q = hand_q_traj;
hand_traj_opts.t = t_q;
hand_traj_opts.interp_mode = 'spline';
hand_traj = HandTraj(hand_traj_opts);

t_samples = linspace(0,T,num_rec_configs);
hand_q_interp_vals = zeros(4, num_rec_configs);
for i = 1:1:length(t_samples)
    hand_q_interp_vals(:, i) = hand_traj.GetHandConfiguration(t_samples(i));
end

sim_inst = ForwardSimulationCombinedStateNewGeometry(pushobj, hand_traj, hand_two_finger, mu);
sim_results = sim_inst.RollOut();
num_rec_configs_sim = size(sim_results.obj_configs, 2);
figure;
hold on;
seg_size = 2;
for i = 1:1:num_rec_configs_sim
    if mod(i, seg_size) == 1 || i == num_rec_configs_sim
         if i == 1
            c= 'k';
        elseif i == num_rec_configs_sim
            c = 'b';
        else
            c = 'r';
         end
    % Plot the object.
        plot(sim_results.obj_configs(1, i), sim_results.obj_configs(2,i), 'b+');
        vertices = SE2Algebra.GetPointsInGlobalFrame(pushobj.shape_vertices, sim_results.obj_configs(:,i));
        vertices(:,end+1) = vertices(:,1);
        plot(vertices(1,:), vertices(2,:), '-', 'Color', c);
        hold on;
        hand_two_finger.Draw(gcf, sim_results.hand_configs(:,i));
    end
end
axis equal;


