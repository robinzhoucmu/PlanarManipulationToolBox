% In local frame: Given multiple point contacts at location pts (2*K) with point
% velocity Vp (2*K), together with coefficient of friction mu, 
% contact outward normal Ct_normal (2*K) and 4th order polynomial coefficients, 
% this function computes the body twist V and applied load F, 
% both normalized by characteristic length pho.
% Vp, Pt, Ct_normal: column vectors.
% Output: solution from the last iteration.
% F: wrench (third component normalized) whose gradient (without scaling factor) equals the V.
% V: twist (third component normalized)
% flag_sol: if 1 then exists a feasible solution; 0 implies jamming and F,V will be all zero vectors. 

function [F, V, flag_sol, flag_converged] = GetVelGivenMultiPtPushPoly4LC(vps, pts, outnormals, mu, pho, coeffs)
flag_sol = 0;
flag_converged = 0;
eps_deltaV = 1e-6;
% Initialize.
V = [0;0;0];
cur_F = 1/sqrt(3)*[1;1;1];
norm_deltaV = 1e+5;
eta = 0.5;
max_iters = 1e+4;
count_iter = 0;
while (norm_deltaV > eps_deltaV) & (count_iter < max_iters)
    A = GetHessianPoly4(cur_F(1), cur_F(2), cur_F(3), coeffs);
    [nxt_F, nxt_V, flag_sol] = GetVelGivenMultiPtPushEllipsoidLC(vps, pts, outnormals, mu, pho, A);
    norm_deltaV = norm(V - nxt_V);
    V = nxt_V;
    cur_F = (1 - eta) * cur_F + nxt_F * eta;
    %cur_F = cur_F - eta2 * H \ nxt_F;
    %cur_F = ScaleForceToOneLevelSet(cur_F, LC_coeffs);
    count_iter = count_iter + 1;
end
F = cur_F;
if (norm_deltaV <= eps_deltaV)
    flag_converged = 1;
end
end
% Jamming.
% coeffs = [ 1.0045    0.9999    1.4538    0.0007    0.0267   -0.0008  0.0279    0.0294 ...
% -0.0132    2.0231    3.8622    3.9103  -0.0234   -0.0936    0.0117 ];
% vps = [1,-1;0,0]; pts = [-1,1;0,0]; outnormals = [-1,1;0,0]; mu = 0.5; pho = 1;
% [F, V, flag_sol, flag_cvg] = GetVelGivenMultiPtPushPoly4LC(vps, pts, outnormals, mu, pho, coeffs)
% Grasping a disc.
% vps = [0, sqrt(3)/2; -1, 0.5;]; pts = [0, -sqrt(3)/2; 1, -0.5];
% outnormals = [0, -sqrt(3)/2;1, -0.5]; mu = 0; pho = 1;
% [F, V, flag_sol, flag_cvg] = GetVelGivenMultiPtPushPoly4LC(vps, pts, outnormals, mu, pho, coeffs)
