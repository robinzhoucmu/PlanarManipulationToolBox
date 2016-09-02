% Input:
% Force,Vel: N*3, row-based examples.
function [err, dev_angle] = EvaluateLinearPredictor(Force, Vel, beta)

pred_vel = Force * beta;
pred_vel_dir = bsxfun(@rdivide, pred_vel, sqrt(sum(pred_vel.^2,2)));
diff_v = pred_vel_dir - Vel;
err = mean(sqrt(sum(diff_v.^2,2)));
dev_angle = mean(acos(diag(Vel * pred_vel_dir')) * 180 / pi);

end

