% Iterative algorithm to solve motion model given sticking contact.
% Use the ellipsoid approximation and call GetVelGivenStickingPtPushEllipsoidLC
% until convergence.
function [F, V] = GetVelGivenStickingPtPushPolyLC(Vp, Pt, LC_coeffs, pho)
eps_deltaV = 1e-5;
% Initialize.
V = [0;0;0];
%cur_F = [1;0;0];
cur_F = 1/sqrt(3)*[1;1;1];
norm_deltaV = 1e+5;
eta = 0.5;
eta2 = 1.0;
while (norm_deltaV > eps_deltaV)
    H = GetHessianPoly4(cur_F(1), cur_F(2), cur_F(3), LC_coeffs);
    [nxt_F, nxt_V] = GetVelGivenStickingPtPushEllipsoidLC(Vp, Pt, H, pho);
    norm_deltaV = norm(V - nxt_V);
    %norm_deltaV = norm(GetVelFrom4thOrderPoly(LC_coeffs, cur_F) - ...
    %                   GetVelFrom4thOrderPoly(LC_coeffs, nxt_F));
    V = nxt_V;
    cur_F = (1 - eta) * cur_F + nxt_F * eta;
    %cur_F = cur_F - eta2 * H \ nxt_F;
    %cur_F = ScaleForceToOneLevelSet(cur_F, LC_coeffs);
end
F = cur_F;
F = ScaleForceToOneLevelSet(F, LC_coeffs);
end

