% Fit a convex 4th order homogenous polynomial.
% Input:
% Force, Vel: 3*N matrix.
% gamma: weight for twist matching.
% beta: weight for wrench matching.
% Output:
% v_{15,1}: for the coeffs of the polynomial.
% xi: surrogate distance to the 1-level set for each input wrench point.
% delta: surrogate velocity alignment error.
% pred_V: predicted twists directions for each input wrench point.
% s: positive projection of twists onto the predicted twist.

function [v, xi, delta, pred_V, s, Q] = Fit4thOrderPolyCVX(Force, Vel, gamma, beta, flag_convex, flag_plot)
lambda = 0.1;
if (nargin == 2) 
    gamma = 1;
    beta = 2;
end
if (nargin == 3)
    beta = 2;
end
% Default is with convexity constraint.
if (nargin < 5)
    flag_convex = 1;
end
if (nargin < 6)
    flag_plot = 1;
end
% Extract row vector.
x = Force(1,:);
y = Force(2,:);
z = Force(3,:);
[d, n] = size(Force);

D = [x.^4; y.^4; z.^4; ... 
     x.^3.*y; x.^3.*z; y.^3.*x; y.^3.*z; z.^3.*x; z.^3.*y; ...
     x.^2.*y.^2; x.^2.*z.^2; y.^2.*z.^2; ...
     x.^2.*y.*z; y.^2.*x.*z; z.^2.*x.*y;]';
 
% Each row in G corresponds to one data point.
G = [x.^3; x.^2.*y; x.^2.*z; x.*y.^2; x.*y.*z; x.*z.^2; y.^3; y.^2.*z; y.*z.^2; z.^3]';
%G = bsxfun(@rdivide, G, sqrt(sum(G.^2,2)));
 %4*v1*x^3 + 3*v4*x^2*y + 3*v5*x^2*z + 2*v10*x*y^2 + 2*v13*x*y*z + 2*v11*x*z^2 + v6*y^3 + v14*y^2*z + v15*y*z^2 + v8*z^3
 %v4*x^3 + 2*v10*x^2*y + v13*x^2*z + 3*v6*x*y^2 + 2*v14*x*y*z + v15*x*z^2 + 4*v2*y^3 + 3*v7*y^2*z + 2*v12*y*z^2 + v9*z^3
 %v5*x^3 + v13*x^2*y + 2*v11*x^2*z + v14*x*y^2 + 2*v15*x*y*z + 3*v8*x*z^2 + v7*y^3 + 2*v12*y^2*z + 3*v9*y*z^2 + 4*v3*z^3    

scaling_min = eps;
 
if (flag_convex == 1)
    cvx_begin quiet
        cvx_precision high
        variable Q(9,9) semidefinite
        variable v(15) %nonnegative
        variables  xi(n) delta(n) s(n) Z(n,3) H(10,3)     
    minimize(lambda * norm(v) + beta * sum(xi)/n + gamma * sum(delta)/n)
    subject to 
        % Point Fitting Constraints.
        H == [4*v(1), v(4), v(5); ...
              3*v(4), 2*v(10), v(13); ...
              3*v(5), v(13), 2*v(11); ...
              2*v(10), 3*v(6), v(14); ...
              2*v(13), 2*v(14), 2*v(15); ...
              2*v(11), v(15), 3*v(8); ...
              v(6), 4*v(2), v(7); ...
              v(14), 3*v(7), 2*v(12); ...
              v(15), 2*v(12), 3*v(9); ...
              v(8), v(9), 4*v(3)]
        for i = 1:n
            norm(D(i,:) * v - 1) <= xi(i) 
            % Predicted vel 1*3.
            Z(i,:) == G(i,:) * H
            norm(Z(i,:) - s(i) * Vel(:,i)') <= delta(i)
            s(i) >= scaling_min
        end
        % Convexity constraints.
        Q(1,1) == 12 * v(1);
        Q(1,2) + Q(2,1) == 6 * v(4);
        Q(1,3) + Q(3,1) == 6 * v(5);
        Q(2,2) == 2 * v(10);
        Q(2,3) + Q(3,2) == 2 * v(13);
        Q(3,3) == 2 * v(11);
        Q(1,4) + Q(4,1) == 6 * v(4);
        Q(1,5) + Q(2,4) + Q(4,2) + Q(5,1) == 8 * v(10);
        Q(1,6) + Q(3,4) + Q(4,3) + Q(6,1) == 4 * v(13);
        Q(2,5) + Q(5,2) == 6 * v(6);
        Q(2,6) + Q(3,5) + Q(5,3) + Q(6,2) == 4 * v(14);
        Q(3,6) + Q(6,3) == 2 * v(15);
        Q(1,7) + Q(7,1) == 6 * v(5);
        Q(1,8) + Q(2,7) + Q(7,2) + Q(8,1) == 4 * v(13);
        Q(1,9) + Q(3,7) + Q(7,3) + Q(9,1) ==  8 * v(11); 
        Q(2,8) + Q(8,2) == 2 * v(14); 
        Q(2,9) + Q(3,8) + Q(8,3) + Q(9,2) == 4 * v(15); 
        Q(3,9) + Q(9,3) == 6 * v(8);  
        Q(4,4) == 2 * v(10); 
        Q(4,5) + Q(5,4) == 6 * v(6); 
        Q(4,6) + Q(6,4) == 2 * v(14); 
        Q(5,5) == 12 * v(2);
        Q(5,6) + Q(6,5) == 6 * v(7); 
        Q(6,6) == 2 * v(12);  
        Q(4,7) + Q(7,4) == 2 * v(13); 
        Q(4,8) + Q(5,7) + Q(7,5) + Q(8,4) == 4 * v(14);
        Q(4,9) + Q(6,7) + Q(7,6) + Q(9,4) == 4 * v(15);
        Q(5,8) + Q(8,5) == 6 * v(7);
        Q(5,9) + Q(6,8) + Q(8,6) + Q(9,5) == 8*v(12);
        Q(6,9) + Q(9,6) == 6*v(9);  
        Q(7,7) == 2*v(11); 
        Q(7,8) + Q(8,7) == 2*v(15); 
        Q(7,9) + Q(9,7) == 6*v(8);
        Q(8,8) == 2*v(12);
        Q(8,9) + Q(9,8) == 6*v(9);
        Q(9,9) == 12*v(3);
    cvx_end
else
    cvx_begin quiet
    cvx_precision high
    variables v(15) xi(n) delta(n) s(n) Z(n,3) H(10,3)     
    minimize(lambda * norm(v) + beta * sum(xi)/n + gamma * sum(delta)/n)
    subject to 
        % Point Fitting Constraints.
        H == [4*v(1), v(4), v(5); ...
              3*v(4), 2*v(10), v(13); ...
              3*v(5), v(13), 2*v(11); ...
              2*v(10), 3*v(6), v(14); ...
              2*v(13), 2*v(14), 2*v(15); ...
              2*v(11), v(15), 3*v(8); ...
              v(6), 4*v(2), v(7); ...
              v(14), 3*v(7), 2*v(12); ...
              v(15), 2*v(12), 3*v(9); ...
              v(8), v(9), 4*v(3)]
        for i = 1:n
            norm(D(i,:) * v - 1) <= xi(i) 
            % Predicted vel 1*3.
            Z(i,:) == G(i,:) * H
            norm(Z(i,:) - s(i) * Vel(:,i)') <= delta(i)
            s(i) >= scaling_min
        end
    cvx_end
end

%  disp('velocity matching error');
%  mean(delta)
%  disp('force matching error');
%  mean(xi)
 pred_V = Z;
 %pred_V_dir = bsxfun(@rdivide, pred_V, sqrt(sum(pred_V.^2, 2)));
 %disp('poly4: velocity direction alignment l2 distance')
 %err = mean(sqrt(sum((pred_V_dir - Vel').^2, 2)))
[err, dev_angle] = EvaluatePoly4Predictor(Force', Vel', v)
if (flag_plot)
    h = Plot4thPoly(v, Force');
    VisualizeForceVelPairs(Force, Vel, h);
end
end

