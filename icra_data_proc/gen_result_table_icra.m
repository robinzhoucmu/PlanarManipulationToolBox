clear all;
%folder_result_save = '~/Pushing/icra_data_proc/motion_model_eval_logs_multirun/plywood_run5_fastvalidation2';
surface_types = {'hardboard', 'plywood'};
%surface_types = {'plywood'};
%surface_types = {'hardboard'};
%shape_ids = {'rect1', 'rect2', 'rect3', 'tri1', 'tri2', 'tri3', 'ellip1', 'ellip2', 'ellip3', 'hex', 'butter'};
%shape_ids = {'3pts1', '3pts2', '3pts3', '3pts4', 'patch'};
shape_ids = {'3pts1', '3pts2', '3pts3', '3pts4'};

ls_types = {'poly4', 'quadratic'};
% Result table for a particular velocity. Each column is a particular object, the odd number rows are surface-poly4
% the even number rows are surface-quad. 
numruns = 10;
result_record_all = cell(numruns, 1);
all_run_avg_disp = zeros(length(surface_types) * length(ls_types), length(shape_ids));
all_run_avg_angle = zeros(length(surface_types) * length(ls_types), length(shape_ids));
all_run_avg_combinedmetric = zeros(length(surface_types) * length(ls_types), length(shape_ids));
all_run_avg_gt_dispnorm_change = zeros(length(surface_types), length(shape_ids));
all_run_avg_gt_angle_change = zeros(length(surface_types), length(shape_ids));
all_run_avg_gt_combinedmetric = zeros(length(surface_types), length(shape_ids));

for ind_run = 1:1:numruns
    result_table_disp_percent = zeros(length(surface_types) * length(ls_types), length(shape_ids));
    result_table_angle_percent = zeros(length(surface_types) * length(ls_types), length(shape_ids));
    result_table_disp = zeros(length(surface_types) * length(ls_types), length(shape_ids));
    result_table_angle = zeros(length(surface_types) * length(ls_types), length(shape_ids));
    result_table_combinedmetric = zeros(length(surface_types) * length(ls_types), length(shape_ids));
    avg_gt_dispnorm_change = zeros(length(surface_types), length(shape_ids));
    avg_gt_angle_change = zeros(length(surface_types), length(shape_ids));
    avg_gt_combinedmetric_change = zeros(length(surface_types), length(shape_ids));
    
    for ind_surface = 1:1:length(surface_types)
        for ind_shape = 1:1:length(shape_ids)
            for ind_ls_type = 1:1:length(ls_types)
                row_id = length(ls_types) * (ind_surface - 1) + ind_ls_type;
                column_id = ind_shape;
                pho = 0.05*sqrt(2);
                str_query = strcat('*', surface_types{ind_surface}, '*', shape_ids{ind_shape}, '*', ...
                    ls_types{ind_ls_type}, '*run', num2str(ind_run), '.mat');
                files = dir(str_query);
                load(files.name);

                result_table_disp(row_id, column_id) = 1000 * avg_diff_disp;
                result_table_angle(row_id, column_id) = (180/pi) * avg_diff_angle;
                result_table_combinedmetric(row_id, column_id) = 1000 * (avg_diff_disp + pho * avg_diff_angle);

                N = length(record_all);
                disp_gt_final = zeros(2, N);
                disp_gt_init = zeros(2, N);
                for i = 1:1:N                                                                     
                    disp_gt_init(:,i) = record_all{i}.init_pose_gt(1:2);
                    disp_gt_final(:,i) = record_all{i}.final_pose_gt(1:2);                       
                end
                angle_gt_changes = compute_angle_diff(angle_gt_init, angle_gt_final);
                angle_dev_percents = abs(angle_diff) ./ abs(angle_gt_changes);


                result_table_disp_percent(row_id, column_id) = 100 * avg_diff_disp / avg_disp_change_norm_gt;
                %avg_diff_angle,avg_angle_change_gt
                result_table_angle_percent(row_id, column_id) = 100 * avg_diff_angle / avg_angle_change_gt ;

                avg_gt_dispnorm_change(ind_surface, ind_shape) = 1000 * avg_disp_change_norm_gt;
                avg_gt_angle_change(ind_surface, ind_shape) = (180/pi) * avg_angle_change_gt;
                avg_gt_combinedmetric_change(ind_surface, ind_shape) = 1000 * ( avg_disp_change_norm_gt + avg_angle_change_gt * pho);
            end
        end
    end

    result_record_all{ind_run}.result_disp_percent = result_table_disp_percent;
    result_record_all{ind_run}.result_angle_percent = result_table_angle_percent;
    result_record_all{ind_run}.result_disp = result_table_disp;
    result_record_all{ind_run}.result_angle = result_table_angle;
    result_record_all{ind_run}.result_combinedmetric = result_table_combinedmetric;
    all_run_avg_disp = all_run_avg_disp + result_table_disp;
    all_run_avg_angle = all_run_avg_angle + result_table_angle;
    all_run_avg_combinedmetric = all_run_avg_combinedmetric + result_table_combinedmetric;
    all_run_avg_gt_dispnorm_change = all_run_avg_gt_dispnorm_change + avg_gt_dispnorm_change;
    all_run_avg_gt_angle_change = all_run_avg_gt_angle_change + avg_gt_angle_change;
    all_run_avg_gt_combinedmetric = all_run_avg_gt_combinedmetric + avg_gt_combinedmetric_change;
    
end
all_run_avg_disp =  all_run_avg_disp / numruns;
all_run_avg_angle =  all_run_avg_angle / numruns;
all_run_avg_combinedmetric = all_run_avg_combinedmetric / numruns;
avg_gt_angle_change = avg_gt_angle_change / numruns;
avg_gt_dispnorm_change =  avg_gt_dispnorm_change / numruns;
all_run_avg_gt_dispnorm_change =  all_run_avg_gt_dispnorm_change / numruns;
all_run_avg_gt_angle_change =  all_run_avg_gt_angle_change / numruns;
all_run_avg_gt_combinedmetric = all_run_avg_gt_combinedmetric / numruns;

% Compute mean and 95% percent confidence.
all_run_avg_disp_cfi = zeros(length(surface_types) * length(ls_types), length(shape_ids));
all_run_avg_angle_cfi = zeros(length(surface_types) * length(ls_types), length(shape_ids));
all_run_avg_combinedmetric_cfi = zeros(length(surface_types) * length(ls_types), length(shape_ids));

for ind_surface = 1:1:length(surface_types)
            for ind_shape = 1:1:length(shape_ids)
                for ind_ls_type = 1:1:length(ls_types)
                    data_disp = zeros(numruns, 1);
                    data_angle = zeros(numruns, 1);
                    data_combinedmetric = zeros(numruns, 1);
                    for ind_run = 1:1:numruns
                        row_id = length(ls_types) * (ind_surface - 1) + ind_ls_type;
                        column_id = ind_shape;
                        data_disp(ind_run) = result_record_all{ind_run}.result_disp(row_id, column_id);
                        data_angle(ind_run) = result_record_all{ind_run}.result_angle(row_id, column_id);
                        data_combinedmetric(ind_run) = result_record_all{ind_run}.result_combinedmetric(row_id, column_id);
                    end
                    alpha = 0.95;
                    [mu, sigma, muci, sigmaci] = normfit(data_disp, 1 - alpha);
                    all_run_avg_disp_cfi(row_id, column_id) = muci(2) - mu;
                    [mu, sigma, muci, sigmaci] = normfit(data_angle, 1 - alpha);
                    all_run_avg_angle_cfi(row_id, column_id) = muci(2) - mu;                    
                    [mu, sigma, muci, sigmaci] = normfit(data_combinedmetric, 1 - alpha);
                    all_run_avg_combinedmetric_cfi(row_id, column_id) = muci(2) - mu;
                end
            end
end

% % Generate column and row labels for latex table generation.
% input.data = zeros(size(all_run_avg_disp, 1), 2 * size(all_run_avg_disp, 2));
% input.data(:,1:2:end) = all_run_avg_disp;
% input.data(:,2:2:end) = all_run_avg_disp_cfi;
% input.data = round(input.data * 100) / 100;
% %input.tableColLabels = shape_ids;
% input.dataFormat = {'%.2f'};
% latex = latexTable(input);
% 
% input.data = zeros(size(all_run_avg_angle, 1), 2 * size(all_run_avg_angle, 2));
% input.data(:,1:2:end) = all_run_avg_angle;
% input.data(:,2:2:end) = all_run_avg_angle_cfi;
% input.data = round(input.data * 100) / 100;
% %input.tableColLabels = shape_ids;
% input.dataFormat = {'%.2f'};
% latex = latexTable(input);

input.data = zeros(size(all_run_avg_combinedmetric, 1), 2 * size(all_run_avg_combinedmetric, 2));
input.data(:,1:2:end) = all_run_avg_combinedmetric;
input.data(:,2:2:end) = all_run_avg_combinedmetric_cfi;
input.data = round(input.data * 100) / 100;
%input.tableColLabels = shape_ids;
input.dataFormat = {'%.2f'};
latex = latexTable(input);

input.data = zeros(size(all_run_avg_gt_combinedmetric));
input.data = all_run_avg_gt_combinedmetric;
input.data = round(input.data * 100) / 100;
input.dataFormat = {'%.2f'};
latex = latexTable(input);

