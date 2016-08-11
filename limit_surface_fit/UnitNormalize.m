% Unit-normalize (row-wise) a row-based data matrix X.
function [X_n] = UnitNormalize(X)
X_n = bsxfun(@rdivide, X, sqrt(sum(X.^2,2)));
end

