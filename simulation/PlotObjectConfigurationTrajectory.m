function [h] = PlotObjectConfigurationTrajectory(sim_results_all, pho, theta_min, theta_max)
if (nargin < 3)
    theta_min = -1e+4;
end
if (nargin < 4)
    theta_max = 1e+4;
end

    h = figure;
    num_poses = length(sim_results_all);
    seg = 4;
    for i = 1:1:num_poses
        if (sim_results_all{i}.obj_configs(3, 1) >= theta_min) & (sim_results_all{i}.obj_configs(3, 1) <= theta_max)
            if ~(mod(i, 5) == 1) continue; end
            traj_obj = sim_results_all{i}.obj_configs; traj_obj(1:2,:) = traj_obj(1:2, :) /pho;
            quiver3(traj_obj(1,1:seg:end-1), traj_obj(2,1:seg:end-1), traj_obj(3,1:seg:end-1), ...
                traj_obj(1,2:seg:end) - traj_obj(1,1:seg:end-1), traj_obj(2,2:seg:end) - traj_obj(2,1:seg:end-1), traj_obj(3,2:seg:end) -traj_obj(3,1:seg:end-1) , ...
                'MaxHeadSize', 0.25)
            %'Color', 'b');
            hold on;
        end
    end
end