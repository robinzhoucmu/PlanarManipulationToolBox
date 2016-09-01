% In local frame: Given a single point contact at location Pt with point
% velocity Vp (with magnitude), together with coefficient of friction, 
% contact normal and parameter for limit surface, this function computes 
% the body twist V and applied load F, both normalized by characteristic length pho.
function [F, V] = ComputeVelGivenSingleContactPtPush(Vp, Pt, Ct_normal, Ct_mu, pho, LC_coeffs, LC_type)
[contact_mode] = DetermineContactModelGivenPtPush(Vp, Pt, Ct_normal, Ct_mu, pho, LC_coeffs, LC_type);
if strcmp(contact_mode,'separation')
    F = zeros(3,1);
    V = zeros(3,1);
elseif strcmp(contact_mode,'sticking')
    if strcmp(lc_type, 'poly4')
        [F, V] = GetVelGivenStickingPtPushPolyLC(Vp, Pt, LC_coeffs, pho);
    elseif strcmp(lc_type, 'quadratic')
        [F, V] = GetVelGivenStickingPtPushEllipsoidLC(Vp, Pt, LC_coeffs, pho);
    end
else
    [fc_edges] = ComputeFrictionConeEdges(Pt, Ct_normal, Ct_mu, pho);
    if strcmp(contact_mode,'leftsliding')
        F = fc_edges(:,1);
        
    else
        F = fc_edges(:,2);
    end
end
end

