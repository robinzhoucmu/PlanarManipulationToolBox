% In local frame: Given multiple point contacts at location pts (2*K) with point
% velocity Vp (2*K), together with coefficient of friction mu, 
% contact outward normal Ct_normal (2*K) and parameter for limit surface, this function computes 
% the body twist V and applied load F, both normalized by characteristic length pho.
% Vp, Pt, Ct_normal: column vectors.
function [F, V, flag_jammed, flag_converged] = ComputeVelGivenMultiContactPtPush(vps, pts, outnormals, mu, pho, ls_coeffs, ls_type)
if strcmp(ls_type, 'quadratic')

elseif strcmp(ls_type, 'poly4')

end
end

