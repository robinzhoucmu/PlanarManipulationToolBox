function [h1,h2] = PlotPrePostDistributions(sim_results_all, pho)
h1 = figure;
h2 = figure;
num_poses = length(sim_results_all);
for ind_pose = 1:1:num_poses
    q_init = sim_results_all{ind_pose}.obj_configs(:,1);
    q_end = sim_results_all{ind_pose}.obj_configs(:,end);
    figure(h1);
    plot3(q_init(1)/pho, q_init(2)/pho, q_init(3), 'b*');
    hold on;
    figure(h2);
    plot3(q_end(1)/pho, q_end(2)/pho, q_end(3), 'r*');
    hold on;
end 
end