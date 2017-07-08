% Read from a folder of logs executing the same sequence of pushes with
% same ICs, expect to end at the same final poses. 
function [key_poses_obj, key_poses_robot, file_names] = read_from_execution_logs(folder_name)
file_infos = dir(fullfile(folder_name, '*.txt'));
num_files = length(file_infos);
key_poses_obj = cell(num_files, 1);
key_poses_robot = cell(num_files, 1);
file_names = cell(num_files, 1);
for i = 1:1:num_files
    file_name = file_infos(i).name;
    data_log = csvread(fullfile(folder_name, file_name));
    % Discard the first row. 
    data_log(1,:) = [];
    file_names{i} = fullfile(folder_name, file_name);
    action_types = data_log(:, end);
    indices_transition = find((action_types(2:end) == action_types(1:end-1))==0);
    indices_transition_all = zeros(length(indices_transition) * 2, 1);
    indices_transition_all(1:2:end) = indices_transition;
    indices_transition_all(2:2:end) = bsxfun(@plus, 1, indices_transition);
    key_poses_obj{i} = [data_log(1,2:4); data_log(indices_transition_all, 2:4); data_log(end, 2:4)]'; 
    key_poses_robot{i} = [data_log(1,5:7); data_log(indices_transition_all, 5:7); data_log(end, 5:7)]'; 
end
end