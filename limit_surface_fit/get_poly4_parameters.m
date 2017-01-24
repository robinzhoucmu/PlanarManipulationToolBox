% Output:
% E(3): the matrix array that maps coefficients in the original poly4 to the
% gradient form poly3.
% A: the matrix array. B = [b_1, ..., b_k].  
% Expressing SOS-Convex matrix constraints: Tr(A_i, Q) = b_k^T*v.
% E,A,B are all sparse.

function [E, A, B] = get_poly4_parameters()
% Number of terms in homogenious poly4.
dim_poly4 = 15;
% Number of terms in gradient form of poly4.
dim_poly3 = 10;
% Number of constraints for enforcing SOS-Convex Hessian.
num_lc = 36;
E = cell(3, 1);
A = cell(num_lc, 1);
B = sparse(dim_poly4, num_lc);

% d = [x^4 y^4 z^4 x^3*y x^3*z y^3*x y^3*z z^3*x z^3*y x^2*y^2 x^2*z^2 y^2*z^2 x^2*y*z y^2*x*z z^2*x*y]
% l = [x.^3; x.^2.*y; x.^2.*z; x.*y.^2; x.*y.*z; x.*z.^2; y.^3; y.^2.*z; y.*z.^2; z.^3]';
% df/dx = 4*v1*x^3 + 3*v4*x^2*y + 3*v5*x^2*z + 2*v10*x*y^2 + 2*v13*x*y*z + 2*v11*x*z^2 + v6*y^3 + v14*y^2*z + v15*y*z^2 + v8*z^3
% df/dy = v4*x^3 + 2*v10*x^2*y + v13*x^2*z + 3*v6*x*y^2 + 2*v14*x*y*z + v15*x*z^2 + 4*v2*y^3 + 3*v7*y^2*z + 2*v12*y*z^2 + v9*z^3
% df/fz = v5*x^3 + v13*x^2*y + 2*v11*x^2*z + v14*x*y^2 + 2*v15*x*y*z + 3*v8*x*z^2 + v7*y^3 + 2*v12*y^2*z + 3*v9*y*z^2 + 4*v3*z^3    

% Compute E1: E1*v = [4v1, 3v4, 3v5, 2v10, 2*v13, 2*v11, v6, v14, v15, v8]^T
% Can use symbolic computation to check for correctness.
E1 = sparse(dim_poly3, dim_poly4);
E1(1,1) = 4;
E1(2,4) = 3;
E1(3,5) = 3;
E1(4,10) = 2;
E1(5,13) = 2;
E1(6,11) = 2;
E1(7,6) = 1;
E1(8,14) = 1;
E1(9,15) = 1;
E1(10,8) = 1;

% Compute E2: E2*v = [v4, 2v10, v13, 3v6, 2v14, v15, 4v2, 3v7, 2v12, v9]^T
E2 = sparse(dim_poly3, dim_poly4);
E2(1,4) = 1;
E2(2,10) = 2;
E2(3,13) = 1;
E2(4,6) = 3;
E2(5,14) = 2;
E2(6,15) = 1;
E2(7,2) = 4;
E2(8,7) = 3;
E2(9,12) = 2;
E2(10,9) = 1;

% Compute E3: E3*v = [v5, v13, 2v11, v14, 2v15, 3v8, v7, 2v12, 3v9, 4v3]^T
E3 = sparse(dim_poly3, dim_poly4);
E3(1,5) = 1; 
E3(2,13) = 1;
E3(3,11) = 2;
E3(4,14) = 1;
E3(5,15) = 2;
E3(6,8) = 3;
E3(7,7) = 1;
E3(8,12) = 2;
E3(9,9) = 3;
E3(10,3) = 4; 

E{1} = E1;
E{2} = E2;
E{3} = E3;
% Compute A,B.
% Q(i,j) = trace(A, Q), where for A: A(j,i) = 1 and all other elements being 0.

ind_val_B = [1, 12;
             4, 6;
             5, 6;
             10, 2;
             13, 2;
             11, 2;
             4, 6;
             10, 8;
             13, 4;
             6, 6;
             14, 4;
             15, 2;
             5, 6;
             13, 4;
             11, 8;
             14, 2;
             15, 4;
             8, 6;
             10, 2;
             6, 6;
             14, 2;
             2, 12;
             7, 6;
             12, 2;
             13, 2;
             14, 4;
             15, 4;
             7, 6;
             12, 8;
             9, 6;
             11, 2;
             15, 2;
             8, 6;
             12, 2;
             9, 6;
             3, 12;
            ];
ind_Q = [1,1;              
        1,2;
        2,1;
        1,3;
        3,1;
        2,2;
        2,3;
        3,2;
        3,3
        1,4;
        4,1;
        1,5;
        2,4;
        4,2;
        5,1;
        1,6;
        3,4;
        4,3;
        6,1;
        2,5;
        5,2;
        2,6;
        3,5;
        5,3;
        6,2;
        3,6;
        6,3;
        1,7;
        7,1;
        1,8;
        2,7;
        7,2;
        8,1;
        1,9;
        3,7;
        7,3;
        9,1;
        2,8;
        8,2;
        2,9;
        3,8;
        8,3;
        9,2;
        3,9;
        9,3;
        4,4;
        4,5;
        5,4;
        4,6;
        6,4;
        5,5;
        5,6;
        6,5;
        6,6;
        4,7;
        7,4;
        4,8;
        5,7;
        7,5;
        8,4;
        4,9;
        6,7;
        7,6;
        9,4;
        5,8;
        8,5;
        5,9;
        6,8;
        8,6;
        9,5;
        6,9;
        9,6;
        7,7;
        7,8;
        8,7;
        7,9;
        9,7;
        8,8;
        8,9;
        9,8;
        9,9;
        ];
    
num_Q = [1;
        2;
        2;
        1;
        2;
        1;
        2;
        4;
        4;
        2;
        4;
        2;
        2;
        4;
        4;
        2;
        4;
        2;
        1;
        2;
        2;
        1;
        2;
        1;
        2;
        4;
        4;
        2;
        4;
        2;
        1;
        2;
        2;
        1;
        2;
        1;
        ];
ind = 1;
for i = 1:1:num_lc
    [A{i}, B(:,i)] = ...
        gen_linear_constraint(ind_Q(ind:ind+num_Q(i) - 1, :), ...
        ind_val_B(i,1), ind_val_B(i,2));
    ind = ind + num_Q(i);
end
% Q(1,1) == 12 * v(1);
% Q(1,2) + Q(2,1) == 6 * v(4);
% Q(1,3) + Q(3,1) == 6 * v(5);
% Q(2,2) == 2 * v(10);
% Q(2,3) + Q(3,2) == 2 * v(13);
% Q(3,3) == 2 * v(11);
% Q(1,4) + Q(4,1) == 6 * v(4);
% Q(1,5) + Q(2,4) + Q(4,2) + Q(5,1) == 8 * v(10);
% Q(1,6) + Q(3,4) + Q(4,3) + Q(6,1) == 4 * v(13);
% Q(2,5) + Q(5,2) == 6 * v(6);
% Q(2,6) + Q(3,5) + Q(5,3) + Q(6,2) == 4 * v(14);
% Q(3,6) + Q(6,3) == 2 * v(15);
% Q(1,7) + Q(7,1) == 6 * v(5);
% Q(1,8) + Q(2,7) + Q(7,2) + Q(8,1) == 4 * v(13);
% Q(1,9) + Q(3,7) + Q(7,3) + Q(9,1) ==  8 * v(11); 
% Q(2,8) + Q(8,2) == 2 * v(14); 
% Q(2,9) + Q(3,8) + Q(8,3) + Q(9,2) == 4 * v(15); 
% Q(3,9) + Q(9,3) == 6 * v(8);  
% Q(4,4) == 2 * v(10); 
% Q(4,5) + Q(5,4) == 6 * v(6); 
% Q(4,6) + Q(6,4) == 2 * v(14); 
% Q(5,5) == 12 * v(2);
% Q(5,6) + Q(6,5) == 6 * v(7); 
% Q(6,6) == 2 * v(12);  
% Q(4,7) + Q(7,4) == 2 * v(13); 
% Q(4,8) + Q(5,7) + Q(7,5) + Q(8,4) == 4 * v(14);
% Q(4,9) + Q(6,7) + Q(7,6) + Q(9,4) == 4 * v(15);
% Q(5,8) + Q(8,5) == 6 * v(7);
% Q(5,9) + Q(6,8) + Q(8,6) + Q(9,5) == 8*v(12);
% Q(6,9) + Q(9,6) == 6*v(9);  
% Q(7,7) == 2*v(11); 
% Q(7,8) + Q(8,7) == 2*v(15); 
% Q(7,9) + Q(9,7) == 6*v(8);
% Q(8,8) == 2*v(12);
% Q(8,9) + Q(9,8) == 6*v(9);
% Q(9,9) == 12*v(3);

end

function [A, b] = gen_linear_constraint(ind_A, ind_b, val_b)
A = sparse(9,9);
n = size(ind_A, 1);
for i = 1:1:n
    ind_r = ind_A(i,2);
    ind_c = ind_A(i,1);
    A(ind_r, ind_c) = 1;
end
b = sparse(15, 1);
b(ind_b) = val_b;
end