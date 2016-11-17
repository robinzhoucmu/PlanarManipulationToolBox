% Interpolote data from get_and_plot_data function output.
function [obj_pose, tip_pt, force, t_q, avg_force] = interp_data(object_pose, tip_pose, wrench, N)
t_s = max([object_pose(1,1), tip_pose(1,1), wrench(1,1)]);
t_e = min([object_pose(end,1), tip_pose(end,1), wrench(end,1)]);
duration = t_e - t_s;
if nargin == 3
    N = size(object_pose,1);
end
%t_q = linspace(t_s, t_e, N);
t_q = linspace(t_s + duration * 0.1, t_e - duration * 0.1, N);
% Sort by first column (time).
object_pose = sortrows(object_pose);
ind_rep_time = find(object_pose(1:end-1,:) == object_pose(2:end,:));
object_pose(ind_rep_time, :) = [];
method = 'linear';
obj_pose = interp1(object_pose(:,1), object_pose(:,2:end), t_q, method);
tip_pt = interp1(tip_pose(:,1), tip_pose(:,2:3), t_q, method);
force = interp1(wrench(:,1), wrench(:,2:3), t_q, method);
avg_force = zeros(size(force));
t_prev = t_q(1);
for i = 1:1:size(obj_pose, 1)
    t_avg = linspace(t_prev, t_q(i), 20);
    avg_force(i,:) = mean(interp1(wrench(:,1), wrench(:,2:3), t_avg));
    t_prev = t_q(i);
end
end