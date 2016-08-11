% Fit elipsoid (e.g., min volumn enclosing elipsoid) 
% given force and velocities using CVX.
% F, V: 3*N matrix. 
% lambda: tradeoff parameter for fitting force.
% gamma: tradeoff parameter for regularization.
% A: x^Ax - 1 = 0;
function [A, xi, delta, pred_V_dir, s] = FitElipsoidForceVelocityCVX(F, V, lambda, gamma, flag_convex, flag_plot)
if nargin <=4
    flag_convex = 1;
end
if nargin <=5
    flag_plot = 1;
end

[d, n] = size(F);
scale_min = eps;
if (flag_convex)
    cvx_begin quiet
        variable A(d,d) semidefinite
        variables xi(n) s(n) delta(n)
    minimize( gamma * norm(A, 'fro') + lambda * sum(xi) + sum(delta))
    subject to
        for i = 1:n
           norm(F(:,i)' * A * F(:,i) - 1) <= xi(i)
           norm(A * F(:,i) - s(i) * V(:,i)) <= delta(i)
           s(i) >= scale_min
    %        norm([0 V(3,i) -V(2,i);
    %                -V(3,i) 0 V(1,i);
    %                V(2,i) -V(1,i) 0] * A * F(:,i)) <= delta(i)
           %(A * F(:,i))' * V(:,i) >= 0
           %norm(A,2) == 1
        end
    cvx_end
else 
    cvx_begin quiet
        variable A(d,d) symmetric
        variables xi(n) s(n) delta(n)
    minimize( gamma * norm(A, 'fro') + lambda * sum(xi) + sum(delta))
    subject to
        for i = 1:n
           norm(F(:,i)' * A * F(:,i) - 1) <= xi(i)
           norm(A * F(:,i) - s(i) * V(:,i)) <= delta(i)
           s(i) >= scale_min
    %        norm([0 V(3,i) -V(2,i);
    %                -V(3,i) 0 V(1,i);
    %                V(2,i) -V(1,i) 0] * A * F(:,i)) <= delta(i)
           %(A * F(:,i))' * V(:,i) >= 0
           %norm(A,2) == 1
        end
    cvx_end 
end
pred_V = A*F;
pred_V_dir = bsxfun(@rdivide, pred_V, sqrt(sum(pred_V.^2)));
%disp('poly2: velocity direction alignment l2 distance')
%err = mean(sqrt(sum((pred_V_dir - V).^2)));
if (flag_plot)
    r = [A(1,1), A(2,2), A(3,3), A(1,2)*2, A(1,3)*2, A(2,3)*2];
    h = DrawEllipsoid(r, F');
    VisualizeForceVelPairs(F, V, h);
end
end