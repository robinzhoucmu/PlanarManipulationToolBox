close all;
pathFolder = {'exp_data/robot_exp_logs/bigrect_3pts', ...
    'exp_data/robot_exp_logs/bigrect_ridge', ...
    'exp_data/robot_exp_logs/butter', ...
    'exp_data/robot_exp_logs/tri'};
datafiles = {'plan_graph_bigrect_mu03nd12_cs03.mat', ...
                    'plan_graph_bigrect_mu03nd12_cs03.mat', ...
                    'plan_graph_butter_mu025nd10_cs05_vbhalfmm.mat', ...
                    'plan_graph_triangle_mu03nd15.mat'};
all_records = cell(12, 1);
for ind_group = 1:1:4
    load(datafiles{ind_group});
    d = dir(pathFolder{ind_group});
    isub = [d(:).isdir]; 
    nameFolds = {d(isub).name}';
    nameFolds(ismember(nameFolds,{'.','..'})) = [];
    % loop over each folder 
    num_folders = length(nameFolds);
    for i = 1:1:num_folders
        folder_name = fullfile(pathFolder{ind_group}, nameFolds{i});
        % process all files.
        [key_poses_obj, key_poses_robot, file_names] = read_from_execution_logs(folder_name);
        table_center = [0; -317.5; 0];
        num_runs = length(key_poses_obj);
        offset_obj_final_poses = zeros(3, num_runs);

        for i = 1:1:num_runs
            offset_obj_final_poses(:,i) = key_poses_obj{i}(:,end) - table_center;
            offset_obj_final_poses(3,i) = offset_obj_final_poses(3,i) * 180 / pi;
        end
        [mu,sigma,muci,sigmaci] = normfit(offset_obj_final_poses', 0.05);
        ind = 4 * (ind_group - 1) + i;
        all_records{ind}.folder_name = folder_name;
        all_records{ind}.avg_offset = mu;
        all_records{ind}.ci = sigmaci(2,:) - mu;
        VisualizePushingExpLog(file_names{num_runs}, pushobj, hand_two_finger);
    end
end