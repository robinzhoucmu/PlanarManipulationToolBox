% Input:
% dir_F and bv (N*3,): example per row.
% coef: coefficients of the poly4.

function [err, dev_angle] = EvaluatePoly4Predictor(dir_F, bv, coef)

pred_vel = GetVelFrom4thOrderPoly(coef, dir_F');
err_v = pred_vel - bv;
err = mean(sqrt(sum(err_v.^2,2)));
angles_test = acos(diag(bv * pred_vel')) * 180 / pi;
dev_angle = mean(angles_test);

end

