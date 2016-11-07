% In local frame: Given multiple point contacts at location pts (2*K) with point
% velocity Vp (2*K), together with coefficient of friction mu, 
% contact outward normal Ct_normal (2*K) and ellipsoid limit surface matrix A, 
% this function computes the body twist V and applied load F, 
% both normalized by characteristic length pho.
% Vp, Pt, Ct_normal: column vectors.
% Output:
% F: wrench (third component normalized) whose gradient (without scaling factor) equals the V.
% V: twist (third component normalized)
% flag_sol: 1 => a feasible moving solution; 0 implies jamming. 

function [F, V, flag_sol] = GetVelGivenMultiPtPushEllipsoidLC(vps, pts, outnormals, mu, pho, A)
num_cts = size(vps, 2);
N = zeros(num_cts, 3);
L = zeros(2*num_cts, 3);
a = zeros(num_cts, 1);
b = zeros(2*num_cts, 1);
E = zeros(2*num_cts, num_cts);
Mus = diag(mu*ones(num_cts,1));
F = zeros(3,1);
V = zeros(3,1);

for i = 1:1:num_cts
    E(2*i-1:2*i, i) = [1;1];
    Jp = [1, 0, -pts(2,i)/pho;
             0, 1, pts(1,i)/pho];
    N(i,:) = - outnormals(:,i)' * Jp;
    % right tangent, left tangent.
    D = [-outnormals(2,i), outnormals(2,i);
            outnormals(1,i), -outnormals(1,i);];
    L(2*i-1:2*i,:) = D' * Jp;
    a(i) = outnormals(:,i)' * vps(:,i);
    b(2*i-1:2*i) = -D' * vps(:,i);
end
q = [a;b;zeros(num_cts,1)];
lcp_M = [N*A*N', N*A*L', zeros(num_cts, num_cts);
               L*A*N', L*A*L', E;
               Mus, -E', zeros(num_cts, num_cts)];
%vps, pts, outnormals
% val_pivot = 1e-5;
% val_miters = 1e+4;
% [w, z, retcode] = LCPSolve(lcp_M, q, val_pivot, val_miters);
% norm(w - lcp_M * z - q)
% if retcode(1) == 1
%     flag_sol = 1;
% elseif (norm(w - lcp_M * z - q) < 1e-3) & (sum(w<0) == 0) & (sum(z<0) == 0) & (w'*z == 0)
%     flag_sol = 1;
% else
%     flag_sol = 0;
% end
z = LCP(lcp_M,q);
w = lcp_M * z + q;
if (norm(w - lcp_M * z - q) < 1e-3) & (sum(w < -1e-3) == 0) & (sum(z < -1e-3) == 0) & (abs(w'*z) <= 1e-3)
    flag_sol = 1;
else
    flag_sol = 0;
end
if (flag_sol)
    fns = z(1:num_cts);
   fts = z(num_cts+1:3*num_cts);
   V = A * (N' * fns + L' * fts);
   F = A \ V;
end
end
% Example 1: Jamming. 
% vps = [1,-1;0,0]; pts = [-1,1;0,0]; outnormals = [-1,1;0,0]; mu = 0.5;
% pho = 1; A = eye(3,3);
% [F, V] = GetVelGivenMultiPtPushEllipsoidLC(vps, pts, outnormals, mu, pho, A)
% Example 2: Two Points Pushing, translation.
% vps = [1,1;0.2,0.2]; pts = [-1,-1;0.5,-0.5]; outnormals = [-1,-1;0,0]; mu = 0.5;
% pho = 1; A = eye(3,3);
% [F, V] = GetVelGivenMultiPtPushEllipsoidLC(vps, pts, outnormals, mu, pho, A)
% Example 3: Two Points Pushing, rotating.
% vps = [1,0.05;0,0]; pts = [-1,-1;0.5,-0.5]; outnormals = [-1,-1;0,0]; mu = 0.5;
% pho = 1; A = eye(3,3);
% [F, V] = GetVelGivenMultiPtPushEllipsoidLC(vps, pts, outnormals, mu, pho, A)
% Example 4: Two Points Pushing while closing
% vps = [1,1;-0.2,0.2]; pts = [-1,-1;0.5,-0.5]; outnormals = [-1,-1;0,0]; mu = 0.5;
% pho = 1; A = eye(3,3);
% [F, V] = GetVelGivenMultiPtPushEllipsoidLC(vps, pts, outnormals, mu, pho, A)
% Exp 5: 2 points grasping disc. not -jamming. increasing mu to tan(pi/6) will jam.
% vps = [0, sqrt(3)/2; -1, 0.5;]; pts = [0, -sqrt(3)/2; 1, -0.5];
% outnormals = [0, -sqrt(3)/2;1, -0.5]; mu = 0; 
% pho = 1; A = eye(3,3);
% [F, V, flag_sol] = GetVelGivenMultiPtPushEllipsoidLC(vps, pts, outnormals, mu, pho, A)

