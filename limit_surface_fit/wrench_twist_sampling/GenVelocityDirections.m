% Input:
% Pts_{2*Np}: x,y coordinates of the discretized pressure distribution 
% CORs_{2*Nc}: x,y coordinates of the center of rotations 
% Output:
% V_(4*Nc)*Np: velocity matrix, where each 2 rows represent the velocities
% of every points in Pts rotating about a particular COR. 
% bv_{2*Nc, 3}: body velocities.
function [V, bv] = GenVelocityDirections(Pts, CORs)
numP = size(Pts, 2);
numC = size(CORs, 2);

% Replicate as matrix (Nc * Np) to avoid for loops.
% Separate the x,y part.
MatPx = repmat(Pts(1,:), [numC 1]);
MatPy = repmat(Pts(2,:), [numC 1]);

MatCx = repmat(CORs(1,:)', [1 numP]);
MatCy = repmat(CORs(2,:)', [1 numP]);

Dx = MatPx - MatCx;
Dy = MatPy - MatCy;

% Compute the velocity inverse norm matrix of size Nc * Np
% Add eps to avoid division of zero.
NormMat = 1./sqrt(Dx .* Dx + Dy .* Dy + eps);

%Too much memory cost
%Dx = NormMat * diag(Pts(1,:)') - diag(CORs(1,:)') * NormMat; 
%Dy = NormMat * diag(Pts(2,:)') - diag(CORs(2,:)') * NormMat;

Dx = bsxfun(@times, NormMat, Pts(1,:)) - bsxfun(@times, NormMat, CORs(1,:)');
Dy = bsxfun(@times, NormMat, Pts(2,:)) - bsxfun(@times, NormMat, CORs(2,:)');

% v = [-dy, dx]; 
% Both clockwise and counter-clockwise.
% Positive sense of rotation.
V = zeros(4*numC, numP);
V(1:2:2*numC, :) = -Dy;
V(2:2:2*numC, :) = Dx;

% Negative sense of rotation.
V(2*numC+1:2:4*numC, :) = Dy;
V(2*numC+2:2:4*numC, :) = -Dx;

% Compute unit/normalized body velocities from COR.
% For COR(xc, yc), the corresponding motion vector is 
normalizer = sqrt(bsxfun(@plus, sum(CORs.^2, 1), 1))';
bv = zeros(2*numC, 3);
% Vx
bv(1:1:end/2,1) = bsxfun(@rdivide, CORs(2,:)', normalizer);
% Vy
bv(1:1:end/2,2) = bsxfun(@rdivide, -CORs(1,:)', normalizer);
% w
bv(1:1:end/2,3) = 1./normalizer;
% Negative rotation sense.
bv(end/2+1:1:end,:) = -bv(1:1:end/2,:);

end


%%Unit test:
% p = [1,2;1,2]
% 
% p =
% 
%      1     2
%      1     2
% 
% V = GenVelocityDirections(p,c)
% 
% V =
% 
%    -0.7071   -0.7071
%     0.7071    0.7071
% 
% c = [0,1;0,1]
% 
% c =
% 
%      0     1
%      0     1
% 
% V = GenVelocityDirections(p,c)
% 
% V =
% 
%    -0.7071   -0.7071
%     0.7071    0.7071
%          0   -0.7071
%          0    0.7071