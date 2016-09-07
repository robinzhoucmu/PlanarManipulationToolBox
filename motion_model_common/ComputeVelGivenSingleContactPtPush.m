% In local frame: Given a single point contact at location Pt with point
% velocity Vp (with magnitude), together with coefficient of friction, 
% contact outward normal Ct_normal and parameter for limit surface, this function computes 
% the body twist V and applied load F, both normalized by characteristic length pho.
% Vp, Pt, Ct_normal: column vectors.
function [F, V, contact_mode] = ComputeVelGivenSingleContactPtPush(Vp, Pt, Ct_normal, Ct_mu, pho, LC_coeffs, LC_type)
[contact_mode] = DetermineContactModelGivenPtPush(Vp, Pt, Ct_normal, Ct_mu, pho, LC_coeffs, LC_type)
if strcmp(contact_mode,'separation')
    F = zeros(3,1);
    V = zeros(3,1);
elseif strcmp(contact_mode,'sticking')
    if strcmp(LC_type, 'poly4')
        [F, V] = GetVelGivenStickingPtPushPolyLC(Vp, Pt, LC_coeffs, pho);
    elseif strcmp(LC_type, 'quadratic')
        [F, V] = GetVelGivenStickingPtPushEllipsoidLC(Vp, Pt, LC_coeffs, pho);
    end
else
    [fc_edges] = ComputeFrictionConeEdges(Pt, Ct_normal, Ct_mu, pho);
    if strcmp(contact_mode,'leftsliding')
        F = fc_edges(:,1);
    else % right sliding.
        F = fc_edges(:,2);
    end
    if strcmp(LC_type, 'poly4')
        V = GetVelFrom4thOrderPoly(LC_coeffs, F)';
    elseif strcmp(LC_type, 'quadratic')
        V = LC_coeffs * F;
    end
    % Compute the right scale of V such that normal velocity is the same.
    B = [1, 0, -Pt(2)/pho;
     0, 1, -Pt(1)/pho];
    s = (Vp' * (-Ct_normal)) / ((B*V)' * (-Ct_normal))
    V = s * V;
end
end

