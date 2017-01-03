% Fit elipsoid (e.g., min volumn enclosing elipsoid) 
% given force and velocities using CVX.
% F, V: 3*N matrix. 
% gamma: tradeoff parameter for aligning twists.
% beta: tradeoff parameter for fitting wrenches.
% A: x^Ax - 1 = 0;
function [A, xi, delta, pred_V_dir, s] = FitEllipsoidForceVelocityCVXOptions(F, V, options)
% Default parameters.
lambda = 0.1;
gamma = 1;
beta = 2;
flag_convex = 1;
flag_plot = 1;
% Parsing options if given.
if nargin == 3
    if isfield(options, 'weight_regularization')
        lambda = options.weight_regularization;
    end
    if isfield(options, 'weight_twist')
        gamma = options.weight_twist;
    end
    if isfield(options, 'weight_wrench')
        beta = options.weight_wrench;
    end
    if isfield(options, 'flag_convex')
        flag_convex = options.flag_convex;
    end
    if isfield(options, 'flag_plot')
        flag_plot = options.flag_plot;
    end
end

[d, n] = size(F);
scale_min = eps;
if (flag_convex)
    cvx_begin quiet
        variable A(d,d) semidefinite
        variables xi(n) s(n) delta(n)
    minimize( lambda * norm(A, 'fro') + beta * sum(xi)/n + gamma * sum(delta)/n)
    subject to
        for i = 1:n
           norm(F(:,i)' * A * F(:,i) - 1) <= xi(i)
           norm(A * F(:,i) - s(i) * V(:,i)) <= delta(i)
           s(i) >= scale_min
        end
    cvx_end
else 
    cvx_begin quiet
        variable A(d,d) symmetric
        variables xi(n) s(n) delta(n)
    minimize(norm(A, 'fro') + beta * sum(xi)/n + gamma * sum(delta)/n)
    subject to
        for i = 1:n
           norm(F(:,i)' * A * F(:,i) - 1) <= xi(i)
           norm(A * F(:,i) - s(i) * V(:,i)) <= delta(i)
           s(i) >= scale_min
        end
    cvx_end 
end
pred_V = A*F;
pred_V_dir = bsxfun(@rdivide, pred_V, sqrt(sum(pred_V.^2)));
% disp('velocity matching surrogate error');
% mean(delta)
% disp('force matching surrogate error');
% mean(xi)
%disp('poly2: velocity direction alignment l2 distance')
%err = mean(sqrt(sum((pred_V_dir - V).^2)))
[err, dev_angle] = EvaluateLinearPredictor(F', V', A)

if (flag_plot)
    r = [A(1,1), A(2,2), A(3,3), A(1,2)*2, A(1,3)*2, A(2,3)*2];
    h = DrawEllipsoid(r, F');
    VisualizeForceVelPairs(F, V, h);
end
end