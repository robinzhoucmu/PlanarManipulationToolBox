% Input: 
% Pts_{2, Np}: x,y coordinates of pressure distribution points. 
% NC: number of samples to create random CORs NOT necessarily on the facet.
% Output:
% CORs_{2, Nc}: x,y coordinates of randomly sampled CORs.
function [CORs] = GenerateRandomCORs3(Pts, Nc, nFacetPts)
if (nargin == 2)
    nFacetPts = max(ceil(Nc/2), 2);
end

numP = size(Pts, 2);
%CORs = zeros([2, Nc + nFacetPts * numP]);

% Compute point center.
pC = mean(Pts, 2);
% Compute average distance to center.
disp = bsxfun(@minus, Pts, pC);
avgR = mean(sqrt(sum(disp.^2)));

CORs_part1 = GenerateRandomCORs2(Nc, avgR);
CORs_part1 = bsxfun(@plus,CORs_part1, pC);
n_part1 = size(CORs_part1,2);
CORs(:,1:n_part1) = CORs_part1;

% Rotation about support points.
CORs(:, n_part1+1:n_part1+ nFacetPts * numP) = repmat(Pts, [1 nFacetPts]);
end

