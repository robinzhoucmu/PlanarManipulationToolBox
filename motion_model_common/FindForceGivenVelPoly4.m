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
    H = GetHessian(fp(1), fp(2), fp(3), coeffs);
    fp = fp - eta* (H \ (vp - V));
    [dir_vp, vp] = GetVelFrom4thOrderPoly(coeffs, fp);
    vp = vp';
    ct = ct + 1;
    err_norm = norm(vp - V);
end
% Scale force to be back on the 1-level set.
x = fp(1); y = fp(2); z = fp(3);
d = [x^4; y^4; z^4; ... 
        x^3*y; x^3*z; y^3*x; y^3*z; z^3*x; z^3*y; ...
        x^2*y^2; x^2*z^2; y^2*z^2; ...
        x^2*y*z; y^2*x*z; z^2*x*y;];
%d'*coeffs
s = (1 / abs((d'*coeffs)))^(1/4);
F = fp*s;
end


function [H] = GetHessian(x,y,z, coeffs)
v1 = coeffs(1);
v2 = coeffs(2);
v3 = coeffs(3);
v4 = coeffs(4);
v5 = coeffs(5);
v6 = coeffs(6);
v7 = coeffs(7);
v8 = coeffs(8);
v9 = coeffs(9);
v10 = coeffs(10);
v11 = coeffs(11);
v12 = coeffs(12);
v13 = coeffs(13);
v14 = coeffs(14);
v15 = coeffs(15);

H = [ 12*v1*x^2 + 6*v4*x*y + 6*v5*x*z + 2*v10*y^2 + 2*v13*y*z + 2*v11*z^2,   3*v4*x^2 + 4*v10*x*y + 2*v13*x*z + 3*v6*y^2 + 2*v14*y*z + v15*z^2,   3*v5*x^2 + 2*v13*x*y + 4*v11*x*z + v14*y^2 + 2*v15*y*z + 3*v8*z^2;
      3*v4*x^2 + 4*v10*x*y + 2*v13*x*z + 3*v6*y^2 + 2*v14*y*z + v15*z^2, 2*v10*x^2 + 6*v6*x*y + 2*v14*x*z + 12*v2*y^2 + 6*v7*y*z + 2*v12*z^2,   v13*x^2 + 2*v14*x*y + 2*v15*x*z + 3*v7*y^2 + 4*v12*y*z + 3*v9*z^2;
      3*v5*x^2 + 2*v13*x*y + 4*v11*x*z + v14*y^2 + 2*v15*y*z + 3*v8*z^2,   v13*x^2 + 2*v14*x*y + 2*v15*x*z + 3*v7*y^2 + 4*v12*y*z + 3*v9*z^2, 2*v11*x^2 + 2*v15*x*y + 6*v8*x*z + 2*v12*y^2 + 6*v9*y*z + 12*v3*z^2];
end
