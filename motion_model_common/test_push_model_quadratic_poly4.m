hex_shape = [0.030250000000000   0.052394536928959
  -0.030250000000000   0.052394536928959
  -0.060500000000000   0.000000000000000
  -0.030250000000000  -0.052394536928959
   0.030250000000000  -0.052394536928959
   0.060500000000000  -0.000000000000000];

% Fitting of hex on plywood.
A =[0.107122349410479  -0.001030543036815   0.001684367614121
  -0.001030543036815   0.101894450314681   0.003114364300318
   0.001684367614121   0.003114364300318   0.179865665670714];

v_poly4_cvx = [0.009624551029737
   0.009532403669269
   0.021675444468091
  -0.000362819729670
   0.000385557292470
   0.000609462532237
   0.000873086822492
   0.004872478599629
   0.006408161014645
   0.017156297048472
   0.045304719170651
   0.029537270025115
   0.000313869849867
  -0.001242833127112
   0.003484458519177];

Vp1 = [0.005; -0.01];
Vp2 = [0.01;0.005];
Vp3 = [0; -0.015];

Pt = ((hex_shape(1,:) + hex_shape(2,:))/2)';
Ct_normal = [0;1];
Ct_mu = 0.25;
pho = compute_shape_avgdist_to_center('hex');

LC_coeffs_quad = A;
LC_type_quad = 'quadratic';
LC_coeffs_poly4 = v_poly4_cvx;
LC_type_poly4 = 'poly4';

[F_quad_1, V_quad_1] = ComputeVelGivenSingleContactPtPush(Vp1, Pt, Ct_normal, Ct_mu, pho, LC_coeffs_quad, LC_type_quad)
[F_poly_1, V_poly_1] = ComputeVelGivenSingleContactPtPush(Vp1, Pt, Ct_normal, Ct_mu, pho, LC_coeffs_poly4, LC_type_poly4)

[F_quad_2, V_quad_2] = ComputeVelGivenSingleContactPtPush(Vp2, Pt, Ct_normal, Ct_mu, pho, LC_coeffs_quad, LC_type_quad)
[F_poly_2, V_poly_2] = ComputeVelGivenSingleContactPtPush(Vp2, Pt, Ct_normal, Ct_mu, pho, LC_coeffs_poly4, LC_type_poly4)

[F_quad_3, V_quad_3] = ComputeVelGivenSingleContactPtPush(Vp3, Pt, Ct_normal, Ct_mu, pho, LC_coeffs_quad, LC_type_quad)
[F_poly_3, V_poly_3] = ComputeVelGivenSingleContactPtPush(Vp3, Pt, Ct_normal, Ct_mu, pho, LC_coeffs_poly4, LC_type_poly4)




