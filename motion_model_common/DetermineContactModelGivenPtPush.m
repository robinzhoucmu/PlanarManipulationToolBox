% Input, output in local frame. Assume that 
% Vp 2*1: the linear velocity (with magnitude) of the pusher point.
% Pt 2*1: the single pusher point xy position.
% Ct_normal: the *outward* normal at the point of contact.
% Ct_mu: the coefficient of friction at the point of contact.
% pho: the chacteristic length used when training the limit surface.
% LC_coeffs: the parameter for limit surface. 
% LC_type: 'quadratic', 'poly4'.

% pho: chacteristic length, has to be consistent with what is used for
% identifying limit surface parameter. 
function [contact_mode] = DetermineContactModelGivenPtPush(Vp, Pt, Ct_normal, Ct_mu, pho, LC_coeffs, LC_type)
% We do not assume pushing by pulling. 
if (Vp' * Ct_normal > 0)
    contact_mode = 'separation';
    return;
end
% Compute applied wrench cone.
[fc_edges] = ComputeFrictionConeEdges(Pt, Ct_normal, Ct_mu, pho);
% Compute body twist motion cone.
[ vc_edges ] = ComputeVelConeGivenFC(fc_edges, LC_coeffs, LC_type);
% Compute point velocity cone.
B = [1, 0, -Pt(2)/pho;
     0, 1, Pt(1)/pho];
vp_cone_edges = B * vc_edges;
% Compute cross product (x1y2 - x2y1) to determine the side. 
k1 = Vp(1) * vp_cone_edges(2,1) - Vp(2) * vp_cone_edges(1,1);
k2 = Vp(1) * vp_cone_edges(2,2) - Vp(2) * vp_cone_edges(1,2);
% if sticking:
if (k1 >=0 && k2 <=0 )
    contact_mode = 'sticking';
elseif (k1 < 0)
    % motion is to the left of the left motion cone edge.
    contact_mode = 'leftsliding';
elseif (k2 > 0)
    % motion is to the right of the right motion cone edge.
    contact_mode = 'rightsliding';
end
end

