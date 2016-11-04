% In local frame: Given multiple point contacts at location pts (2*K) with point
% velocity Vp (2*K), together with coefficient of friction mu, 
% contact outward normal Ct_normal (2*K) and parameter for limit surface, this function computes 
% the body twist V and applied load F, both normalized by characteristic length pho.
% Vp, Pt, Ct_normal: column vectors.
% Output wrench F is scaled back to the 1-limit surface. 

function [F, V, flag_jammed, flag_converged] = ComputeVelGivenMultiContactPtPush(vps, pts, outnormals, mu, pho, ls_coeffs, ls_type)
if strcmp(ls_type, 'quadratic')
    [F, V, flag_jammed] = GetVelGivenMultiPtPushEllipsoidLC(vps, pts, outnormals, mu, pho, ls_coeffs);
    s = F'*ls_coeffs*F;
    F = F / (sqrt(s));
elseif strcmp(ls_type, 'poly4')
    [F, V, flag_jammed, flag_converged] = GetVelGivenMultiPtPushPoly4LC(vps, pts, outnormals, mu, pho, ls_coeffs);
    F = ScaleForceToOneLevelSet(F, ls_coeffs);
else
    fprintf('Limit surface %s type not recognized\n', ls_type);
end
end

