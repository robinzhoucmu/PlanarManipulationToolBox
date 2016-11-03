% In local frame: Given multiple point contacts at location pts (2*K) with point
% velocity Vp (2*K), together with coefficient of friction mu, 
% contact outward normal Ct_normal (2*K) and ellipsoid limit surface matrix A, 
% this function computes the body twist V and applied load F, 
% both normalized by characteristic length pho.
% Vp, Pt, Ct_normal: column vectors.
function [F, V] = GetVelGivenMultiPtPushEllipsoidLC(vps, pts, outnormals, mu, pho)
num_cts = size(vps, 2);
N = zeros(num_cts, 3);
L = zeros(2*num_cts, 3);
a = zeros(num_cts, 1);
b = zeros(2*num_cts, 1);
E = zeros(2*num_cts, num_cts);
for i = 1:1:num_cts
    E(2*i-1:2*i, i) = [1;1];
    Jp = 
    N()
end



end

