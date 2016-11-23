folder_result_save = '~/Pushing/mcube_data_proc/motion_model_eval_logs_train_test_searchmu2_filter/wrench1twist1_20percenttraining';
vels = [10];
surface_types = {'abs', 'plywood'};
shape_ids = {'rect1', 'rect2', 'rect3', 'tri1', 'tri2', 'tri3', 'ellip1', 'ellip2', 'ellip3', 'hex', 'butter'};
ls_types = {'poly4', 'quadratic'};
% Result table for a particular velocity. Each column is a particular object, the odd number rows are surface-poly4
% the even number rows are surface-quad. 

result_table_disp = zeros(2 * length(ls_types), length(shape_ids));
result_table_angle = zeros(2 * length(ls_types), length(shape_ids));
for ind_vel = 1:1:length(vels)
    for ind_surface = 1:1:length(surface_types)
        for ind_shape = 1:1:length(shape_ids)
            for ind_ls_type = 1:1:length(ls_types)
                row_id = 2 * (ind_surface - 1) + ind_ls_type;
                column_id = ind_shape;
                str_query = strcat('*', surface_types{ind_surface}, '*', shape_ids{ind_shape}, '*', ...
                    ls_types{ind_ls_type}, '*', num2str(vels), '*.mat')
                files = dir(str_query);
                load(files.name);
                result_table_disp(row_id, column_id) = avg_diff_disp / avg_disp_change_norm_gt;
                avg_diff_angle,avg_angle_change_gt
                result_table_angle(row_id, column_id) = avg_diff_angle / avg_angle_change_gt;
            end
        end
    end
end
