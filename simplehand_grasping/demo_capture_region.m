tic;
shape_info.shape_id = 'circle1';
shape_info.shape_type = 'circle';
shape_info.shape_parameters.radius = 0.02;
shape_info.pho = shape_info.shape_parameters.radius;

options_support_pts.mode = 'circle';
options_support_pts.range = shape_info.shape_parameters.radius;

num_supports_pts = 100; 
support_pts = GridSupportPoint(num_supports_pts, options_support_pts); % N*2.

options_pressure.mode = 'uniform';
pressure_weights = AssignPressure(support_pts, options_pressure);

ls_coeff = [ 1.0276 
    %1.0381
    1.0276
    5.1589
   -0.0300
    0.4683
   -0.0498
   -0.4392
    0.5807
   -0.5780
    2.0527
    7.3867
    7.3668
   -0.3526
    0.3345
   -0.1457
];
ls_type = 'poly4';

pushobj = PushedObject(support_pts', pressure_weights, shape_info, ls_type, ls_coeff);

ls_coeff_quad = diag([1.1417, 1.0792, 2.4776]);
ls_type_quad = 'quadratic';
pushobj = PushedObject(support_pts', pressure_weights, shape_info, ls_type_quad, ls_coeff_quad);


pitch_fun = @(t)(3*pi);

mu = 0.05;
finger_radius = 0.002;
ratio_uncertainty = 2.0
num_init_pose_samples = 600;
flag_stop_at_first_contact = 1;
[all_results, simulation_inst] = ComputeThreeFingersCircleCaptureRegionPitchFunction(pushobj, pitch_fun, mu, ratio_uncertainty, finger_radius, num_init_pose_samples, flag_stop_at_first_contact);
figure;
hold on;
for i = 1:1:length(all_results)
    if (all_results{i}.result_flags.jammed == 1) 
        plot(all_results{i}.pose_log(1,end), all_results{i}.pose_log(2,end), 'k*');
    elseif (all_results{i}.result_flags.missed == 1)
        plot(all_results{i}.pose_log(1,end), all_results{i}.pose_log(2,end), 'bo');
    else
        plot(0,0,'r+');
    end
end
title('post distribution');
axis equal;
toc;