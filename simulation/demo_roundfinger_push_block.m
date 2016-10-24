tic;
%setup_simluation_roundfingerblock;
load roundfinger_block.mat;
pushobj.pose= [0;0;0];
sim_inst = ForwardSimulation(pushobj, hand_traj, hand_single_finger, mu, dt_collision);
[sim_results] = sim_inst.RollOut();