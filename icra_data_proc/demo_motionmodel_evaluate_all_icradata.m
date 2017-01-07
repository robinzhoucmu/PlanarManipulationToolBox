clear all; 
close all;
rng(100);
folder_result_save = '~/Pushing/icra_data_proc/motion_model_eval_logs_multirun/idealls_mu05';
flag_use_ideal_ls = 1;

log_files_set = {};
 log_files_set{5} = 'SensorLogs/30_63.33_43.33_30_43.33_63.33/exp_08_17_50.txt';
 log_files_set{6} = 'SensorLogs/10_130_10_10_10_130/exp_08_17_50.txt';
 log_files_set{7} = 'SensorLogs/30_90_30_30_30_90/exp_08_11_50_mixed.txt';
 log_files_set{8} = 'SensorLogs/10_90_10_10_30_130/exp_08_15_50.txt';
 %log_files_set{10} = 'SensorLogs/patch/exp_08_18_1435_50.txt';
% 
log_files_set{1} = 'SensorLogs/wood_30_63.33_43.33_30_43.33_63.33/exp_08_18_1330_50.txt';
log_files_set{2} = 'SensorLogs/wood_10_130_10_10_10_130/exp_08_17_50.txt';
log_files_set{3} = 'SensorLogs/wood_30_90_30_30_30_90/exp_08_17_0922_50.txt';
log_files_set{4} = 'SensorLogs/wood_10_90_10_10_30_130/exp_08_18_1100_50.txt';
%log_files_set{5} = 'SensorLogs/wood_patch/exp_08_17_0839_50.txt';

Tri_pts{1} = [0.03, 0.06333, 0.04333; 0.03, 0.04333, 0.06333];
Tri_pts{2} = [0.01, 0.01, 0.13; 0.01,0.13,0.01];
Tri_pts{3} = [0.03    0.09    0.03; 0.03    0.03    0.09];
Tri_pts{4} = [0.01, 0.09,0.01;0.01,0.03,0.13];

surface_types = {'plywood', 'hardboard'};
%surface_types = {'plywood'};
%surface_types = {'hardboard'};
%pressure_ids = {'3pts1', '3pts2', '3pts3', '3pts4', 'patch'};
pressure_ids = {'3pts1', '3pts2', '3pts3', '3pts4'};
%pressure_ids = {'3pts1'};
ls_types = {'poly4', 'quadratic'};

Tri_mass = 1.518;
mu_f = 4.2 / (Tri_mass * 9.8);
Tri_total_pressure = Tri_mass * mu_f;
Tri_com = [0.15/3; 0.15/3];

ratio_train = 0.5;
ratio_val = 0;
mu = 0.5;
num_runs = 5;
rho =  0.05 * sqrt(2);
for ind_run = 1:1:num_runs
    for ind_surface = 1:1:length(surface_types)
        for ind_pressure = 1:1:length(pressure_ids)
              for ind_ls_type = 1:1:length(ls_types)
                log_id = (ind_surface - 1) * length(pressure_ids) + ind_pressure;
                % Use the same seed for different ls_types so that they
                % train and test on the same random split.
                rng((log_id - 1) * num_runs + ind_run);
                file_name = log_files_set{log_id};
                ls_type = ls_types{ind_ls_type};
                pressure_id = pressure_ids{ind_pressure};
                surface_type = surface_types{ind_surface};
                if (~flag_use_ideal_ls)
                %[record_all, record_ls_training] = EvaluatePositionOffsetICRAData(file_name, ls_type, mu, ratio_train, ratio_val);                
                [record_all, record_ls_training] = EvaluatePositionOffsetICRADataMotionValidation(...
                    file_name, ls_type, mu, ratio_train, ratio_val);
                else
                    tri_pts = bsxfun(@minus, Tri_pts{ind_pressure}, Tri_com);
                    [pds] = GetObjParaFromSupportPts(tri_pts, [0;0], Tri_total_pressure);
                    [record_all, record_ls_training] = EvaluatePositionOffsetICRADataGivenPressure(...
                        file_name, ls_type, mu, ratio_train, ratio_val, tri_pts, pds);    
                end
                N = length(record_all);
                diff_trans = zeros(2, N);
                angle_gt_final = zeros(N,1);
                angle_gt_init = zeros(N,1);
                disp_gt_final = zeros(2, N);
                disp_gt_init = zeros(2, N);
                angle_sim_final = zeros(N,1);
                for i = 1:1:N
                    diff_trans(:,i) = record_all{i}.final_pose_gt(1:2) - record_all{i}.final_pose_sim(1:2);
                    angle_gt_init(i) = mod(record_all{i}.init_pose_gt(3) + 2*pi, 2*pi);
                    angle_gt_final(i) = mod(record_all{i}.final_pose_gt(3) + 2*pi, 2*pi);
                    disp_gt_init(:,i) = record_all{i}.init_pose_gt(1:2);
                    disp_gt_final(:,i) = record_all{i}.final_pose_gt(1:2);
                    angle_sim_final(i) = mod(record_all{i}.final_pose_sim(3) + 2*pi, 2*pi);
                end
                file_save = strcat(folder_result_save, surface_type, '_', pressure_id, '_', ls_type, '_run', num2str(ind_run));
                avg_angle_change_gt = mean(abs(compute_angle_diff(angle_gt_init, angle_gt_final)));
                avg_disp_change_norm_gt = mean(sqrt(sum((disp_gt_final - disp_gt_init).^2, 1)));
                avg_diff_disp = mean(sqrt(sum(diff_trans.^2, 1)));
                angle_diff = compute_angle_diff(angle_sim_final, angle_gt_final);
                avg_diff_angle = mean(abs(angle_diff));
                ls_type
                avg_diff_combinedmetric = avg_diff_disp + rho * avg_diff_angle

                avg_change_gt_combinedmetric = avg_disp_change_norm_gt + rho * avg_angle_change_gt 
                %num_training_pairs = size(record_ls_training.wrenches, 1)
                save(file_save, 'record_all', 'angle_diff', 'angle_gt_init', 'angle_gt_final', 'angle_sim_final', ...
                      'avg_diff_disp', 'avg_diff_angle' , 'avg_diff_combinedmetric', 'avg_change_gt_combinedmetric', ...
                      'surface_type', 'pressure_id',  'ls_type', 'mu', 'avg_angle_change_gt', 'avg_disp_change_norm_gt', ...
                      'record_ls_training');
              end
        end
    end
end