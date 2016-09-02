function [H] = GetHessianPoly4(x,y,z, coeffs)
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