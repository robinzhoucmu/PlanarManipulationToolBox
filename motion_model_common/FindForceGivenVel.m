% V: generalized velocity, normalized.
% coeffs for mode 'quadratic' is the A matrix.
% coeffs for mode 'poly4' is the 15 number coefficients.
function [F] = FindForceGivenVel(V, coeffs, mode)
if strcmp(mode, 'poly4')
    F = FindForceGivenVelPoly4(V, coeffs);
elseif strcmp(mode, 'quadratic')
    % F = (inv(A) * V) / sqrt(V'*inv(A) * V)
    A = coeffs;
    T = A \ V;
    F = T / sqrt(V' * T);
end

end

