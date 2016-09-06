folder_name = '~/Pushing/pushing_data'; query_info.surface='plywood'; 
query_info.shape = 'rect2'; query_info.velocity = 20; num_samples_perfile = 25; 
%[all_wrenches_local, all_twists_local] = read_json_files(folder_name, query_info, num_samples_perfile);
all_twists_local_normalized = UnitNormalize(all_twists_local);
rng(1);
num_sample_pairs = 30;
sampledindices = datasample(1:1:size(all_wrenches_local,1), num_sample_pairs,'Replace',false);

[v_s_cvx, xi, delta, pred_V, s] = Fit4thOrderPolyCVX(all_wrenches_local(sampledindices,:)', ...
    all_twists_local_normalized(sampledindices,:)', 1, 1, 1, 1);

disp('test')
[err, dev_angle] = EvaluatePoly4Predictor(all_wrenches_local, all_twists_local_normalized, v_s_cvx)

[v_s, xi, delta, pred_V, s] = Fit4thOrderPolyCVX(all_wrenches_local(sampledindices,:)', ...
    all_twists_local_normalized(sampledindices,:)', 1, 1, 0, 1);

disp('test')
[err, dev_angle] = EvaluatePoly4Predictor(all_wrenches_local, all_twists_local_normalized, v_s)

[A_s, xi, delta, pred_V, s] = FitEllipsoidForceVelocityCVX(all_wrenches_local(sampledindices,:)', ...
        all_twists_local_normalized(sampledindices,:)', 1, 1, 1, 1);

disp('test')
[err, dev_angle] = EvaluateLinearPredictor(all_wrenches_local, all_twists_local_normalized, A_s)
