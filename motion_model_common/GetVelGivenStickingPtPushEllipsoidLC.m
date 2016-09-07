% Input, output in local frame. Assume that 
% Vp 2*1: the linear velocity (with magnitude) of the pusher point.
% Pt 2*1: the pusher point xy position.
% A: the ellipsoid parameter for limit surface. 
% pho: chacteristic length, has to be consistent with what is used for
% identifying limit surface parameter. 
% ----
% F: the wrench/generalized force with the 3rd component torque/pho. 
% V: the body twist with the 3rd component angular velocity * pho. V has
% magnitude such that it matches with the given linear velocity.

function [F, V] = GetVelGivenStickingPtPushEllipsoidLC(Vp, Pt, A, pho)
%Formulate constraint on body twist. COR has to line on the line 
%orthogonal to the pushing point velocity.
% Vp_x = V_1 - (V_3/pho) * Pt_y
% Vp_y = V_2 + (V_3/pho) * Pt_x
% Equivalent to Vp = B*V
B = [1, 0, -Pt(2)/pho;
     0, 1, -Pt(1)/pho];
% The third component of the applied wrench is a cross product.
% F_3 = (- Pt_y * F_x + Pt_x * F_y)/pho
c = [-Pt(2)/pho, Pt(1)/pho, -1];
% V = A*F, and then concatenate to form the linear system.
D = [B*A;c];
d = [Vp;0];
F = D \ d;
V = A*F;
% Scale F back to 1-level set. 
s = F'*A*F;
F = F / (sqrt(s));
end

