function [v] = GetPoly4CoefficientFromDecompositionMatrix(Q, A, B)
num_A = length(A);
y = zeros(num_A, 1);
for i = 1:1:num_A
    y(i) = trace(A{i} * Q); 
end
v = (B') \ y;
end