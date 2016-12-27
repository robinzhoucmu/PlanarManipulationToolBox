% Helper function that computes the corresponding homogenious
% transformation of given translation and quaternion.
% Input are column vectors.
function [H] = getHomogTransf(trans, quat)
H = zeros(4,4);
H(1:3, 1:3) = qGetR(quat);
H(1:3, 4) = trans;
H(4,4) = 1;
end


