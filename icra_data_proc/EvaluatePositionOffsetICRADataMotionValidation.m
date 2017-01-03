% ratio_train is the ratio of the entire dataset for training. 
% ratio_validation is the ratio among training dataset for validation
% search over parameters.
% file_name is the sensor log file that contains multiple (50ish) pushes to a
% particular pressure arrangement and contact surface.
function [record_all, record_ls_training] = EvaluatePositionOffsetICRADataMotionValidation(file_name, ls_type, mu, ratio_train, ratio_validation)
p = gcp;
if (isempty(p))
    num_physical_cores = feature('numcores');
    parpool(num_physical_cores);
end
trans = [50;50;0];
% Local frame transformation w.r.t mocap local frame (lower left corner).
H_tf = [eye(3,3), trans;
      0,0,0,1];
% Tool transform.
R_tool = [sqrt(2)/2, sqrt(2)/2;
          sqrt(2)/2, -sqrt(2)/2]';
% Parameters for trianglular block.         
le = 0.15;
% Formula for computing rho using wolfram alpha:
% sqrt((\int_{0}^{0.15}(\int_{0}^{0.15-x}(x^2+y^2)dy)dx - 0.5*0.15*0.15*0.05^2)/(0.5*0.15*0.15));
% mass is 0.5*0.15*0.15 assuming density equals 1 and then use parallel
% axis theorem. 
Tri_pho = le * sqrt(2);
unit_scale = 1000;
% Construct triangular push object.
shape_info.shape_id = 'tri';
shape_info.shape_type = 'polygon';

% lower left, upper left, lower right. 
shape_info.shape_vertices = [-le/3, -le/3, le*2/3;
                             -le/3, le*2/3, -le/3];
shape_info.pho = Tri_pho;
% Note that the object 2d pose is already at the center of the object.
[record_log] = ExtractFromLog(file_name, Tri_pho, R_tool, H_tf, unit_scale);
                         
tip_radius = 0.001;
hand_single_finger = ConstructSingleRoundFingerHand(tip_radius);


%Split train_all and test trials. 
num_trials = size(record_log.push_wrenches, 1);
index_perm = randperm(num_trials);
split_ind = ceil(num_trials * ratio_train);
index_train = index_perm(1:split_ind);
index_test = index_perm(split_ind + 1:end);

options.flag_convex = 1;
options.est_mu = mu;
options.flag_dir = 0;
options.method = ls_type;

[info] = CrossValidationSearchParametersMotionModel(shape_info, tip_radius, record_log, index_train, ratio_validation, options)
ls_coeffs = info.coeffs;
record_ls_training.mu = info.mu;
record_ls_training.ls_coeffs = ls_coeffs;
record_ls_training.ls_type = ls_type;

 [avg_combined_metric_test, record_all] = EvalCombinedMetricGivenFileListICRA(...
               record_log, index_test, ls_type, ls_coeffs, shape_info, hand_single_finger, ...
               record_ls_training.mu); 
fprintf('test: metric %f\n', avg_combined_metric_test);
end