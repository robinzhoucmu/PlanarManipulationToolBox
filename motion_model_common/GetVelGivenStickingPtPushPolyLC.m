% Iterative algorithm to solve motion model given sticking contact.
% Use the ellipsoid approximation and call GetVelGivenStickingPtPushEllipsoidLC
% until convergence.
function [F, V] = GetVelGivenStickingPtPushPolyLC(Vp, Pt, LC_coeffs, pho)
eps_deltaF = 1e-4;
% Initialize.
cur_F = [1;0;0];
norm_deltaF = 1e-5;
while (norm_deltaF > eps_deltaF)
    H = GetHessian(cur_F(1), cur_F(2), cur_F(3), LC_coeffs); 
    [nxt_F, V] = GetVelGivenStickingPtPushEllipsoidLC(Vp, Pt, H, pho);
    norm_deltaF = norm(nxt_F - cur_F);
    cur_F = nxt_F;
end
F = cur_F;
end

