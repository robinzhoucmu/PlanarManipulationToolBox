% Input: 
% fc_edges: 3*(2N) edges of the friction cone, angular component already multiplied with pho.
% lc_coeffs: limit surface coefficients.
% if 'gp', lc_coeffs would be a struct containing training data and
% gp parameters.
% lc_type: 'quadratic', 'poly4' or 'gp'.
% Output:
% vc_edges: 3*(2N) edges of the velocity cone, with torque normalized.
function [ vc_edges ] = ComputeVelConeGivenFC(fc_edges, lc_coeffs, lc_type)
if strcmp(lc_type, 'quadratic')
    A = lc_coeffs;
    vel = A * fc_edges;
    dir_vel = bsxfun(@rdivide, vel, sqrt(sum(vel.^2, 1)));
    vc_edges = dir_vel;
elseif strcmp(lc_type, 'poly4')
    [dir_vel, vel] = GetVelFrom4thOrderPoly(lc_coeffs, fc_edges);
    vc_edges = dir_vel';
elseif strcmp(lc_type, 'gp')
    F_train = lc_coeffs.F_train;
    V_train = lc_coeffs.V_train;
    coeffs = lc_coeffs.coeffs;
    [hyp, dev_angle_train, dev_angle, v_gp_test] = GP_Fitting(F_train, V_train, fc_edges', [], coeffs);
    vc_edges = v_gp_test';
end
end

