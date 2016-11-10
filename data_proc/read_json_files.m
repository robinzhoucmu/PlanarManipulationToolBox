% folder_name: the json folder that contains all the related json files
% collected my MIT MCube lab. 
% For detailed description of the dataset: https://mcube.mit.edu/push-dataset
% query_info: specify multiple fields.
% 1) surface: 'plywood', ''
% 2) shape: 'rect1', 'butter', 'rect2', etc.
% 3) velocity: '10',
% It retrieves concatenated wrench and twist data from the jscon files that
% match the query_info.

% Sample usage: folder_name = '~/Pushing/pushing_data'; query_info.surface='abs'; 
% query_info.shape = 'rect1'; query_info.velocity = 10; num_samples_perfile = 8; 
% [all_wrenches_local, all_twists_local] = read_json_files(folder_name, query_info, num_samples_perfile);
function [ all_wrenches_local, all_twists_local, all_vel_tip_local, all_dists, all_vel_slip] = read_json_files(folder_name, query_info, num_samples_perfile)
str_folder = strcat(folder_name, '/', query_info.surface, '/', query_info.shape, '/')
%'motion_surface=plywood_shape=rect1_a=0_v=10_i=0.000_s=0.000_t=0.349.json'
str_file = strcat('motion_surface=', query_info.surface, '_shape=', query_info.shape, ...
                  '*v=', num2str(query_info.velocity), '_i*.json')
listing = dir(strcat(str_folder, str_file));    
% Remove all near-empty files.
for i = 1:1:length(listing)
    if listing(i).bytes < 100
        delete(strcat(str_folder,listing(i).name));
    end
end
addpath(str_folder);
listing = dir(strcat(str_folder, str_file));    
num_files = length(listing);
all_wrenches_local = zeros(num_files * num_samples_perfile, 3);
all_twists_local = zeros(num_files * num_samples_perfile, 3);
all_vel_tip_local = zeros(num_files * num_samples_perfile, 2);
all_dists = zeros(num_files * num_samples_perfile, 1);
all_vel_slip = zeros(num_files * num_samples_perfile, 1);
shape_vertices = get_shape(query_info.shape);
shape_vertices(end,:) = [];
[pho] = compute_shape_avgdist_to_center(query_info.shape);
for i = 1:1:num_files
    (i+0.0)/num_files
    file_name = listing(i).name
    [object_pose, tip_pose, wrench] = get_and_plot_data(file_name, query_info.shape, 0);
    size(object_pose)
    N = num_samples_perfile + 3;
    [obj_pose, tip_pt, force, t_q] = interp_data(object_pose, tip_pose, wrench, N);
    [wrench_local, twist_local, vel_tip_local, dists, vel_slip] = compute_wrench_twist_local_frame(shape_vertices, force, obj_pose, tip_pt, t_q);
    wrench_local(:,3) = wrench_local(:,3) / pho;
    twist_local(:,3) = twist_local(:,3) * pho;
    all_wrenches_local((i-1) * num_samples_perfile+1: i*num_samples_perfile, :) = wrench_local(2:end-1,:);
    all_twists_local((i-1) * num_samples_perfile+1: i*num_samples_perfile, :) = twist_local(2:end-1, :);
    all_vel_tip_local((i-1) * num_samples_perfile+1: i*num_samples_perfile, :) = vel_tip_local(2:end-1, :);
    all_dists((i-1) * num_samples_perfile+1: i*num_samples_perfile, :) = dists(2:end-1);
    all_vel_slip((i-1) * num_samples_perfile+1: i*num_samples_perfile, :) = vel_slip(2:end-1);
end
%Remove close-to-static moves.
index_static = find(sqrt(sum(all_twists_local.^2,2)) < 0.001);
all_wrenches_local(index_static,:) = [];
all_twists_local(index_static,:) = [];
all_vel_tip_local(index_static, :) = [];
all_dists(index_static) = [];
all_vel_slip(index_static) = [];
%Remove small wrenches.
avg_wrench_norm = mean(sqrt(sum(all_wrenches_local.^2,2)));
index_small_wrenches = find(sqrt(sum(all_wrenches_local.^2,2)) < avg_wrench_norm * 0.5);
index_big_wrenches = find(sqrt(sum(all_wrenches_local.^2,2)) > avg_wrench_norm * 2);
all_wrenches_local([index_small_wrenches;index_big_wrenches],:) = [];
all_twists_local([index_small_wrenches;index_big_wrenches],:) = [];
all_vel_tip_local([index_small_wrenches;index_big_wrenches], :) = [];
all_dists([index_small_wrenches;index_big_wrenches]) = [];
all_vel_slip([index_small_wrenches;index_big_wrenches], :) = [];
%all_twists_local_normalized = UnitNormalize(all_twists_local);

figure; plot3(all_wrenches_local(:,1), all_wrenches_local(:,2), all_wrenches_local(:,3), 'r.');


end

