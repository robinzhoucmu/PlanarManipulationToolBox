% Generate random CORs from randomly sampled unit velocities.
function [CORs] = GenerateRandomCORs2(Nv, pho)
% Rejection sample Nv number of velocities.
Ns = ceil(Nv * (8 / (4*pi/3))); 
%Sample from [-1,1] cube.
t = bsxfun(@minus, rand(Ns, 3), [0.5,0.5,0.5]) * 2;
% Rejection sampling.
ind = sum(t.^2, 2) <= 1;
v = t(ind,:);
num_v = size(v,1);
% Project back to the sphere.
v = bsxfun(@rdivide, v, sqrt(sum(v.^2, 2)));

CORs = zeros(2, num_v);
CORs(1,:) = -bsxfun(@rdivide, v(:,2), v(:,3));
CORs(2,:) = bsxfun(@rdivide, v(:,1), v(:,3));

CORs = CORs * pho;
end

