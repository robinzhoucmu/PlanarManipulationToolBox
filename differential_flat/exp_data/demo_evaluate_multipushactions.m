%folder_name = '~/dubins_pushing/bigrect_3pts_20_neg30_90';
folder_name = '~/dubins_pushing/bigrect_ridge_20_neg30_90';
%load plan_graph_bigrect_mu03nd12_cs03.mat;

%folder_name = '~/dubins_pushing/tri_0_0_90';
%load plan_graph_triangle_mu03nd15.mat;

folder_name = '~/dubins_pushing/butter_60_20_135';
load plan_graph_butter_mu025nd10_cs05_vbhalfmm.mat;

[key_poses_obj, key_poses_robot, file_names] = read_from_execution_logs(folder_name);
table_center = [0; -317.5; 0];
num_runs = length(key_poses_obj);
offset_obj_final_poses = zeros(3, num_runs);

for i = 1:1:num_runs
    offset_obj_final_poses(:,i) = key_poses_obj{i}(:,end) - table_center;
    offset_obj_final_poses(3,i) = offset_obj_final_poses(3,i) * 180 / pi;
end
[mu,sigma,muci,sigmaci] = normfit(offset_obj_final_poses', 0.05);

VisualizePushingExpLog(file_names{i}, pushobj, hand_two_finger);

