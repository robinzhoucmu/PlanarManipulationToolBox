tic;
shape_info.shape_id = 'circle1';
shape_info.shape_type = 'circle';
shape_info.shape_parameters.radius = 0.02;
%shape_info.pho = shape_info.shape_parameters.radius;

options_support_pts.mode = 'rim';
ratio_support_rim = 1.0;
options_support_pts.range = ratio_support_rim * shape_info.shape_parameters.radius;
shape_info.pho = ratio_support_rim * shape_info.shape_parameters.radius;

num_supports_pts = 2; 
%support_pts = GridSupportPoint(num_supports_pts, options_support_pts); % N*2.
support_pts = SampleSupportPoint(num_supports_pts, options_support_pts);
options_pressure.mode = 'uniform';
%options_pressure.mode = 'random';
pressure_weights = AssignPressure(support_pts, options_pressure);

ls_coeff = [   1.004487776488424
   1.005250998296516
   5.354783646156310
   0.002398361293719
   0.000000000001541
   0.008249800764895
   0.000000000000036
   0.150706348334593
   0.000000000000066
   2.004813856921745
   7.455281550335771
   7.435194683091581
   0.000000000000073
   0.021749766452242
   0.078381957959822
];


ls_type = 'poly4';
a =  shape_info.shape_parameters.radius * [0, sqrt(2)/2];
b =  shape_info.shape_parameters.radius  * (-a/norm(a));
support_pts = [a;b]
pressure_weights = [norm(b)/(norm(a)+norm(b));(1-norm(b)/(norm(a)+norm(b)))]
shape_info.pho = [norm(a),norm(b)]*pressure_weights;
figure, plot(support_pts(:,1), support_pts(:,2), '^');
axis equal;
drawnow;
pushobj = PushedObject(support_pts', pressure_weights, shape_info, ls_type, ls_coeff);
pushobj.FitLS('poly4', 400, 0.1);

 %ls_coeff_quad = diag([1.1417, 1.0792, 2.4776*10]);
 %ls_type_quad = 'quadratic';
 %pushobj = PushedObject(support_pts', pressure_weights, shape_info, ls_type_quad, ls_coeff_quad);


pitch_fun = @(t)(0*pi);

mu = 0;
finger_radius = 0.002;
ratio_uncertainty = 2.0;
num_init_pose_samples = 200;
flag_stop_at_first_contact = 0;
[all_results, simulation_inst] = ComputeThreeFingersCircleCaptureRegionPitchFunction(...
    pushobj, pitch_fun, mu, ratio_uncertainty, finger_radius, num_init_pose_samples, flag_stop_at_first_contact);
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