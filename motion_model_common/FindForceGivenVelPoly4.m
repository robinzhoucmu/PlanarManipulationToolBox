function [ F ] = FindForceGivenVelPoly4( V, coeffs)
%angle = [0;0];
%[angle, err] = fminsearch(@(angle)EvaluateForceDirectionPoly4(angle, coeffs, V), [0;0]);
%options = optimoptions('Display', 'off');

% [angle, err] = fmincon(@(angle)EvaluateForceDirectionPoly4(angle, coeffs, V), [0;0], [], [], [], [], [0;0], [2*pi; pi])
% % Find the scale that maps f to 1-level set of poly4. 
% theta = angle(1);
% phi = angle(2);
% f = [cos(theta)*sin(phi); sin(theta)*sin(phi); cos(phi)];
% x = f(1); y = f(2); z = f(3);
% d = [x^4; y^4; z^4; ... 
%         x^3*y; x^3*z; y^3*x; y^3*z; z^3*x; z^3*y; ...
%         x^2*y^2; x^2*z^2; y^2*z^2; ...
%         x^2*y*z; y^2*x*z; z^2*x*y;];
% %d'*coeffs
% s = (1 / abs((d'*coeffs)))^(1/4);
% F = f*s;
eta = 0.5;
fp = [1;0;0];
[dir_vp, vp] = GetVelFrom4thOrderPoly(coeffs, fp);
vp = vp';
eps_norm = 1e-5;
ct = 1;
err_norm = norm(vp - V); 
while ( err_norm > eps_norm)
    H = GetHessianPoly4(fp(1), fp(2), fp(3), coeffs);
    fp = fp - eta* (H \ (vp - V));
    [dir_vp, vp] = GetVelFrom4thOrderPoly(coeffs, fp);
    vp = vp';
    ct = ct + 1;
    err_norm = norm(vp - V);
end
% Scale force to be back on the 1-level set.
% x = fp(1); y = fp(2); z = fp(3);
% d = [x^4; y^4; z^4; ... 
%         x^3*y; x^3*z; y^3*x; y^3*z; z^3*x; z^3*y; ...
%         x^2*y^2; x^2*z^2; y^2*z^2; ...
%         x^2*y*z; y^2*x*z; z^2*x*y;];
% %d'*coeffs
% s = (1 / abs((d'*coeffs)))^(1/4);
% F = fp*s;
[F] = ScaleForceToOneLevelSet(fp, coeffs);

end



