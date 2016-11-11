function [] = EvaluatePositionOffsetMCubeData(folder_name, surface_type, shape_id, vel, num_samples_perfile)
rng(1);
query_info.surface= surface_type; 
query_info.shape = shape_id; 
query_info.velocity = vel; 
if (nargin < 5)
    num_samples_perfile = 20;
end
[all_wrenches_local, all_twists_local, vel_tip_local, dists, vel_slip] = read_json_files(folder_name, query_info, num_samples_perfile); 
all_twists_local_normalized = UnitNormalize(all_twists_local);
num_sample_pairs = 500;
sampledindices = datasample(1:1:size(all_wrenches_local,1), num_sample_pairs,'Replace',false);
[v_s_cvx, xi, delta, pred_V, s] = Fit4thOrderPolyCVX(all_wrenches_local(sampledindices,:)', ...
    all_twists_local_normalized(sampledindices,:)', 1, 1, 1, 1);



end