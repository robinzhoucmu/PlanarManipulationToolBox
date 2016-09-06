% Input:
% V_{2*Nc, Np}: The unit/normalized velocities of each PD point about #Nc CORs.
% Pts_{2,Np}: The x,y coordinates for each PD point.
% Pd_{Np,1}: The support force at each PD point.
% Output:
% F_{3, Nc}: The forces and moment (about the COM) for each CORs. 

% For point of zero velocity, we randomly assign a force direction for it.

function [F] = GetFrictionForce(V, Pts, PD)
NumC = size(V,1) / 2;
F = zeros(3, NumC);
% Compute the COM.
%COM = Pts * PD / sum(PD);

% Compute the forces matrix V_{2*Nc, Np}
MatF = V * diag(PD);

% When body is rotating w.r.t to one of the support points, 
% its force could be in any direction and any scale(0-1) since  
% We randomly select a direction for that point.

for i = 1:1:NumC
   ind = find(V(2*i-1,:) == 0 & V(2*i, :) == 0);
   if (~isempty(ind))
       F_rnd = rand(2,1) - [0.5;0.5];
       F_rnd = rand(1,1) * PD(ind) * F_rnd / sqrt(sum(F_rnd.^2));
       MatF(2*i-1:2*i, ind) = F_rnd;
   end
end
% Represent coords with respect to COM. Disp_{2,Np}
%Disp = bsxfun(@minus, Pts, COM);

% Compute moment with respect to the point of origin.
Disp = bsxfun(@minus, Pts, [0;0]);

% Compute the moment MatM_{Nc, Np} 
MatM = bsxfun(@times, Disp(1,:), MatF(2:2:2*NumC, :)) + ...
       bsxfun(@times, Disp(2,:), -MatF(1:2:2*NumC, :));

% Sum over each row tF_{2Nc,1}. Odd number of rows are sum of Fx.
tF = sum(MatF, 2);
F(1,:) = tF(1:2:2*NumC);
F(2,:) = tF(2:2:2*NumC);
% Add the moment term.
F(3,:) = sum(MatM, 2);

end

% Test case
% V
% 
% V =
% 
%    -0.7071   -0.7071
%     0.7071    0.7071
%          0   -0.7071
%          0    0.7071
% 
% p
% 
% p =
% 
%      1     2
%      1     2
% 
% pd
% 
% pd =
% 
%      2
%      1
% 
% F = GetFrictionForce(V, p, pd)
% 
% F =
% 
%    -2.1213   -0.7071
%     2.1213    0.7071
%     0.0000    0.9428
% Note that the first moment (3,1) is not zero if friction is not zero at
% the zero velocity point.

