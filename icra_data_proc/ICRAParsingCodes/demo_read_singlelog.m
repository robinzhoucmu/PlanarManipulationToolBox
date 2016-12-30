file_name = 'SensorLogs/10_130_10_10_10_130/exp_08_17_50.txt';
trans = [50;50;0];
H_tf = [eye(3,3), trans;
      0,0,0,1];
R_tool = [sqrt(2)/2, sqrt(2)/2;
          sqrt(2)/2, -sqrt(2)/2]';


% Parameters for trianglular block.         
Tri_mass = 1.518;
% Black board.
mu_f_blk = 4.2 / (Tri_mass * 9.8);
% Wood board.
mu_f_wood = 5.0 / (Tri_mass * 9.8);
Tri_com = [0.15/3; 0.15/3];
Tri_pho = 0.05;
unit_scale = 1000;

% Note that the object 2d pose is already at the center of the object.
[record_log] = ExtractFromLog(file_name, Tri_pho, R_tool, H_tf, unit_scale);

% Construct triangular push object.
shape_info.shape_id = 'tri';
shape_info.shape_type = 'polygon';
le = 0.15;
% lower left, upper left, lower right. 
shape_info.shape_vertices = [-le/3, -le/3, le*2/3;
                             -le/3, le*2/3, -le/3];
shape_info.pho = Tri_pho;
                         
tip_radius = 0.001;
hand_single_finger = ConstructSingleRoundFingerHand(tip_radius);
ind_trial_test = 2;
% Specify finger trajectory.
hand_poses = record_log.robot_2d_pos{ind_trial_test}';
hand_traj_opts = [];
hand_traj_opts.q = hand_poses;
hand_traj_opts.t = linspace(0,1, size(hand_poses, 2));
hand_traj_opts.interp_mode = 'spline';
hand_traj = HandTraj(hand_traj_opts);

weight_wrench = 1;
weight_twist = 1;
wrenches_train = record_log.push_wrenches';
twists_train = record_log.slider_velocities';
ls_type = 'quadratic';
if strcmp(ls_type, 'poly4')
    [ls_coeffs, xi, delta, pred_V, s] = Fit4thOrderPolyCVX(wrenches_train, twists_train, weight_twist, weight_wrench, 1, 1);
else strcmp(ls_type, 'quadratic')
    [ls_coeffs, xi, delta, pred_V, s] = FitEllipsoidForceVelocityCVX(wrenches_train, twists_train, weight_twist, weight_wrench, 1, 1);
end

pushobj = PushedObject([], [], shape_info, ls_type, ls_coeffs);

object_poses = record_log.obj_2d_traj{ind_trial_test}';

pushobj.pose = object_poses(:,1);
mu = 1.0;
sim_inst = ForwardSimulationCombinedState(pushobj, hand_traj, hand_single_finger, mu);
[sim_results] = sim_inst.RollOut();

weight_angle_to_disp = 1;
alpha = mod(sim_results.obj_configs(3,end) + 10 * pi, 2*pi);
beta = mod(object_poses(3, end) + 10 *pi, 2*pi);
dev =  norm(sim_results.obj_configs(1:2,end) - object_poses(1:2, end)) + ...
       weight_angle_to_disp * pushobj.pho * abs(compute_angle_diff(alpha, beta))
   
   
%% Plotting
num_rec_configs = size(sim_results.obj_configs, 2);
figure;
hold on;
seg_size = 10;
for i = 1:1:num_rec_configs
    if mod(i, seg_size) == 1
    % Plot the square object.
        plot(sim_results.obj_configs(1, i), sim_results.obj_configs(2,i), 'b+');
        vertices = SE2Algebra.GetPointsInGlobalFrame(pushobj.shape_vertices, sim_results.obj_configs(:,i));
        vertices(:,end+1) = vertices(:,1);
        plot(vertices(1,:), vertices(2,:), 'r-');
     % Plot the round point pusher.
        drawCircle(sim_results.hand_configs(1,i), sim_results.hand_configs(2,i), hand_single_finger.finger_radius, 'k');
    end
end
axis equal;