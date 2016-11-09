function [h1,h2] = PlotPrePostDistributions(sim_results_all, pho, theta_min, theta_max)
if (nargin < 3)
    theta_min = -1e+4;
end
if (nargin < 4)
    theta_max = 1e+4;
end
h1 = figure;
h2 = figure;
num_poses = length(sim_results_all);
ct_roc = 0;
ct_tot = 0;
for ind_pose = 1:1:num_poses
    if (sim_results_all{ind_pose}.obj_configs(3,1) >= theta_min) & (sim_results_all{ind_pose}.obj_configs(3,1) <= theta_max)
        ct_tot = ct_tot + 1;
        q_init = sim_results_all{ind_pose}.obj_configs(:,1);
        q_end = sim_results_all{ind_pose}.obj_configs(:,end);
        q_init(1:2) = q_init(1:2) / pho;
        q_end(1:2) = q_end(1:2) / pho;
        if (norm(q_end(1:2)) < 1e-2) & ((abs(q_end(3)) < 0.075) | (abs(q_end(3) - 2*pi/3) < 0.075))
            color = 'r';
            ct_roc = ct_roc + 1;
        else
            color = 'k';
        end
        figure(h1);
        plot3(q_init(1), q_init(2), q_init(3), '.', 'Color', color);
        hold on;
        figure(h2);
        plot3(q_end(1), q_end(2), q_end(3), '.', 'Color', color);
        hold on;
    end
end
ct_roc, ct_tot
ct_roc / ct_tot
end