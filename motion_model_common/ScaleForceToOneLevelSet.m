function [F] = ScaleForceToOneLevelSet(F, coeffs)
% Scale force to be back on the 1-level set.
x = F(1); y = F(2); z = F(3);
d = [x^4; y^4; z^4; ... 
        x^3*y; x^3*z; y^3*x; y^3*z; z^3*x; z^3*y; ...
        x^2*y^2; x^2*z^2; y^2*z^2; ...
        x^2*y*z; y^2*x*z; z^2*x*y;];
%d'*coeffs
s = (1 / abs((d'*coeffs)))^(1/4);
F = F*s;

end

