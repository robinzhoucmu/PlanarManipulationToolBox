% Iterative algorithm to solve motion model given sticking contact.
% Use the ellipsoid approximation and call GetVelGivenStickingPtPushEllipsoidLC
% until convergence.
% Same input, output format as GetVelGivenStickingPtPushEllipsoidLC. 
function [F, V] = GetVelGivenStickingPtPushPolyLC(Vp, Pt, LC_coeffs, pho)
%eps_deltaV = 1e-5;
eps_deltaV = 1e-2;
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
% F = ScaleForceToOneLevelSet(F, LC_coeffs);

% FminCon with trust-region method (to avoid saddle points).
% The optimization is formulates as follows:
% minimize_F || B \grad{H(F)} - Vp ||
% s.t. c^F = 0;
%F0 = 1/sqrt(3)*[1;1;1];
% Initialize with the previous iterative algorithm.
F0 = F;
B = [1, 0, -Pt(2)/pho;
     0, 1, Pt(1)/pho];
c = [-Pt(2)/pho, Pt(1)/pho, -1];
% Interesting Note: trust region reflective get stuck in saddle points with
% 1/sqrt(3)*[1;1;1] initialization.
options = optimoptions('fmincon', 'Display', 'off', 'GradObj', 'on', 'Algorithm', 'trust-region-reflective', 'TolFun', 1e-8);
[F_unscaled, res] = fmincon(@(F)PointVelocityAlignmentResidualPoly4(...
    F, LC_coeffs, Vp, B), F0, [], [], c, 0, [], [], [], options);
[dir_V, V] = GetVelFrom4thOrderPoly(LC_coeffs, F_unscaled);
V = V';
F = ScaleForceToOneLevelSet(F_unscaled, LC_coeffs);
end

