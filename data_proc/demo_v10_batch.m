close all; clear all;
% Get all the json logs 
listing = dir('pushing_data/rect1_json/*v=10_i*.json');
num_files = length(listing);
num_samples = 8;
all_wrenches_local = zeros(num_files * num_samples, 3);
all_twists_local = zeros(num_files * num_samples, 3);
for i = 1:1:num_files
(i+0.0)/num_files
file_name = listing(i).name
[object_pose, tip_pose, wrench] = get_and_plot_data(file_name,'rect1', 0);
N = num_samples + 3;
[obj_pose, tip_pt, force, t_q] = interp_data(object_pose, tip_pose, wrench, N);
[wrench_local, twist_local] = compute_wrench_twist_local_frame(force, obj_pose, tip_pt, t_q);
pho = 0.05;
wrench_local(:,3) = wrench_local(:,3) / pho;
twist_local(:,3) = twist_local(:,3) * pho;
all_wrenches_local((i-1) * num_samples+1: i*num_samples, :) = wrench_local(2:end-1,:);
all_twists_local((i-1) * num_samples+1: i*num_samples, :) = twist_local(2:end-1, :);
end
%Remove close-to-static moves.
index_static = find(sqrt(sum(all_twists_local.^2,2)) < 0.001);
all_wrenches_local(index_static,:) = [];
all_twists_local(index_static,:) = [];
%Remove small wrenches.
index_small_wrenches = find(sqrt(sum(all_wrenches_local.^2,2)) < 0.5);
all_wrenches_local(index_small_wrenches,:) = [];
all_twists_local(index_small_wrenches,:) = [];

all_twists_local_normalized = UnitNormalize(all_twists_local);

figure; plot3(all_wrenches_local(:,1), all_wrenches_local(:,2), all_wrenches_local(:,3), 'r.');