% The objective function value and gradient for position control motion model. 
% (B * \grad H(F) - Vp)^T * (B * \grad H(F) - Vp)
% Input: current wrench point F (3*1), poly4 coefficient coeffs, contact point 
% velocity Vp (2*1) to match with, velocity constraint matrix B (2*3). 
function [res, grad] = PointVelocityAlignmentResidualPoly4(F, coeffs, Vp, B, pho)
[dir_pred_v, pred_v] = GetVelFrom4thOrderPoly(coeffs, F);
pred_v = pred_v';
v_offset = B * pred_v - Vp;
res = v_offset' * v_offset;
H = GetHessianPoly4(F(1), F(2), F(3), coeffs);
grad = 2 * H * B' * v_offset;
end