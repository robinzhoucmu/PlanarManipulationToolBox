% Given alpha, beta (vectors) in [0, 2*pi). Compute the minimum angle of rotation
% to move alpha to beta? 
function [angle_diff] = compute_angle_diff(alpha, beta)
angle_diff = zeros(size(alpha));
angle_diff_ccw = mod(bsxfun(@plus, beta - alpha, 2*pi), 2*pi);
angle_diff_cw = mod(bsxfun(@plus, alpha - beta, 2*pi), 2*pi);
ind_ccw = angle_diff_ccw < angle_diff_cw;
angle_diff(ind_ccw) = angle_diff_ccw(ind_ccw);
angle_diff(~ind_ccw) = -angle_diff_cw(~ind_ccw);
end