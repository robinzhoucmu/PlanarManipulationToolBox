tic;
setup_simluation_roundfingerblock;
%load roundfinger_block.mat;
pushobj.pose= [0;0;0];
sim_inst = ForwardSimulation(pushobj, hand_traj, hand_single_finger, mu, dt_collision);
[sim_results] = sim_inst.RollOut();

% plotting. 
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
        drawCircle(sim_results.hand_configs(1,i), sim_results.hand_configs(2,i), hand_single_finger.finger_radius, 'k');
    end
end
axis equal;