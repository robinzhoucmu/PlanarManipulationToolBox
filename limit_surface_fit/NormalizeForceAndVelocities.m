% Erdman's normalization by characteristic length.
% Input: 
% V, F: N*3, 
% pho: characteristic length pho.
% Output:
% V,F,F_dir: N*3.
function [V, F] = NormalizeForceAndVelocities(V, F, pho)
F(:,3) = F(:,3) / pho;
V(:,3) = V(:,3) * pho;
V = bsxfun(@rdivide, V, sqrt(sum(V.^2,2)));

end

