% Input: 
% shape_info for pushobject.
% record_logs from ExtractFromLog function (icra data)
% indices_train_all: indices for training (including data later partitioned for validation)
% ratio_validation: the ratio among training all for validation.
% Output: parameter stored after the search. 
% options include the ls_type, limit surface training and estimated
% coefficient of friction to search for.
function [para] = CrossValidationSearchParametersMotionModel(shape_info, tip_radius, record_log, indices_train_all, ratio_validation, options)

wrenches = record_log.push_wrenches';
twists = record_log.slider_velocities';

wrenches_train_all = wrenches(:, indices_train_all);
twists_train_all = twists(:, indices_train_all);

% Split train_all into train and validation.
[twists_train, twists_val, wrenches_train, wrenches_val, indices_train_select, indices_val_select] = ...
            SplitTrainTestData(twists_train_all', wrenches_train_all', 1 - ratio_validation);
indices_train = indices_train_all(indices_train_select);
indices_val = indices_train_all(indices_val_select);

hand_single_finger = ConstructSingleRoundFingerHand(tip_radius);
mu = options.est_mu;
%mu_trials = [mu-0.1; mu - 0.05; mu; mu + 0.05; mu+0.1];
mu_trials = [mu - 0.1; mu; mu+0.1];

flag_plot = 0;
flag_convex = options.flag_convex;
method = options.method;

%w_force = [0.1, 0.25, 1];
%w_force = [0.5, 1, 2, 4];
w_force = [0.1, 0.5, 2];
w_vel = [1];
%w_reg = [1, 2, 4, 8];
w_reg = [0.1, 0.5, 4];

best_mu = 0;
best_w_force = -1;
best_w_vel = -1;
best_w_reg = 0;
best_combined_metric = 1e+9;
% For a given w_force, w_vel combinations, train 
% the ls_coeffs, then search for the best mu on training data. 
% Pick the best (ls_coeffs, mu) on validation data.
 for ind_f = 1:length(w_force)
    for ind_v = 1:length(w_vel)
        for ind_r = 1:length(w_reg)
            option_train.weight_regularization = w_reg(ind_r);
            option_train.weight_twist = w_vel(ind_v);
            option_train.weight_wrench = w_force(ind_f);
            option_train.flag_convex = 1; option_train.flag_plot = 0;
           if (strcmp(method, 'poly4'))   
                %[coeffs, xi, delta, pred_v_train, s] = ...
                %   Fit4thOrderPolyCVX(wrenches_train', twists_train', w_vel(ind_r), w_force(ind_f), flag_convex, flag_plot);
               [coeffs, xi, delta, pred_v_train, s] = ...
                   Fit4thOrderPolyCVXOptions(wrenches_train', twists_train', option_train);
               % Evaluate on validation set.
               [err ,dev_angle] = EvaluatePoly4Predictor(wrenches_val, twists_val, coeffs);
               [train_err, train_dev_angle] = EvaluatePoly4Predictor(wrenches_train, twists_train, coeffs);
               fprintf('poly4: w_force:%f, w_vel:%f, w_reg:%f, dev_angle_train:%f, dev_angle_val:%f\n', w_force(ind_f), w_vel(ind_v), w_reg(ind_r), train_dev_angle, dev_angle);

           elseif (strcmp(method, 'quadratic'))
               %[coeffs, xi_elip, delta_elip, pred_v_lr_train, s_lr] = ...
               %    FitEllipsoidForceVelocityCVX(wrenches_train', twists_train',  w_vel(ind_v), w_force(ind_f),  flag_convex, flag_plot);
              [coeffs, xi_elip, delta_elip, pred_v_lr_train, s_lr] = ...
                   FitEllipsoidForceVelocityCVXOptions(wrenches_train', twists_train',  option_train);
               [err, dev_angle] = EvaluateLinearPredictor(wrenches_val, twists_val, coeffs);
               [train_err, train_dev_angle] = EvaluateLinearPredictor(wrenches_train, twists_train,  coeffs);
               fprintf('quadratic: w_force:%f, w_vel:%f, w_reg:%f, dev_angle_train:%f, dev_angle_val:%f\n', w_force(ind_f), w_vel(ind_v), w_reg(ind_r), train_dev_angle, dev_angle);
           end
           % Search for the best mu on the training set.
           mu_train = 0;
           ct_mu = 1;
           metric_best_train = 1e+3;
           ls_coeffs = coeffs;
           ls_type = method;
           while ct_mu <= length(mu_trials)
               mu_candidate = mu_trials(ct_mu);
               [avg_combined_metric] = EvalCombinedMetricGivenFileListICRA(...
                   record_log, indices_train, ls_type, ls_coeffs, shape_info, hand_single_finger, ...
                   mu_candidate);  
               if (avg_combined_metric < metric_best_train)
                    metric_best_train = avg_combined_metric;
                    mu_train = mu_candidate;
                end
                ct_mu = ct_mu + 1;
           end
           fprintf('%s, wrench: %f, twist: %f, reg: %f,  mu_trial: %f, metrictrain: %f \n', ...
               ls_type, option_train.weight_wrench, option_train.weight_twist, option_train.weight_regularization, mu_train, metric_best_train); 
           % Use (ls_coeffs, mu) pair to evaluate on the validation set.
          [avg_combined_metric_val] = EvalCombinedMetricGivenFileListICRA(...
                   record_log, indices_val, ls_type, ls_coeffs, shape_info, hand_single_finger, ...
                   mu_train); 
          fprintf('%s, wrench: %f, twist: %f, reg: %f,  mu_trial: %f, metricval: %f \n', ...
              ls_type, option_train.weight_wrench, option_train.weight_twist, option_train.weight_regularization, mu_train, avg_combined_metric_val); 
           % Update the best so far according to the average combined metric
           % on validation data.
           if (best_combined_metric > avg_combined_metric_val) 
                best_w_force = w_force(ind_f);
                best_w_vel  = w_vel(ind_v);
                best_w_reg = w_reg(ind_r);
                best_mu = mu_train;
                best_coeffs = ls_coeffs;
                best_combined_metric = avg_combined_metric_val;
           end
        end
    end
 end
    para.coeffs = best_coeffs;
    para.mu = best_mu;
    para.w_force = best_w_force;
    para.w_vel = best_w_vel;
    para.w_reg = best_w_reg;
    para.best_combined_metric_val = best_combined_metric;
    display(para);
    fprintf('%s, w_forcebest: %f, w_velbest: %f, w_regbest: %f,  mubest: %f, metricvalbest: %f \n', ...
        ls_type, best_w_force, best_w_vel, best_w_reg, best_mu, best_combined_metric); 
end