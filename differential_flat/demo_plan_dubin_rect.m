rng(1);
tic;
%% Construct pushobj.
% shape and support points.
shape_info.shape_id = 'polygon1';
shape_info.shape_type = 'polygon';
le = 0.02;
% CCW.
shape_info.shape_vertices = [-le,le,le/2,-le;-le,-le,le,le];
shape_info.pho = le;                                            
% Uniformly sample points in the polygon area 
options_support_pts.mode = 'polygon';
options_support_pts.vertices = shape_info.shape_vertices';
num_support_pts = 20;

support_pts = GridSupportPoint(num_support_pts, options_support_pts); % N*2.
num_support_pts = size(support_pts, 1);
options_pressure.mode = 'uniform';
pressure_weights = AssignPressure(support_pts, options_pressure);
%figure, plot(support_pts(:,1), support_pts(:,2), '^');
%axis equal;
%drawnow;

% limit surface fitting based on pressure distribution.
ls_type = 'quadratic';
% Uncomment the following two lines if you first run this file. 
%pushobj = PushedObject(support_pts', pressure_weights, shape_info, ls_type);
%pushobj.FitLS(ls_type, 200, 0.1);
pushobj.noise_df = 10000;
A = pushobj.ls_coeffs;
a = A(1,1);
b_normalized = A(3,3);
%pushobj.ls_coeffs = diag([a;a;b_normalized]);
b = b_normalized / (pushobj.pho^2);

tip_radius = le / 10;
hand_single_finger = ConstructSingleRoundFingerHand(tip_radius);
r = le + tip_radius;

mu = 0.3;
parameters.a = a;
parameters.b = b;
parameters.r = r;
parameters.mu = mu;
pose_start = [0;0;0];
pose_end = [0;le;pi];
[x, y, theta, u, z] = GetDubinPath(pose_start, pose_end, parameters);

num_rec_configs = length(x);
figure;
hold on;
seg_size = 20;
for i = 1:1:num_rec_configs
    if mod(i, seg_size) == 1 || i == num_rec_configs
    % Plot the object.
        plot(x(i), y(i), 'b+');
        obj_pose = [x(i);y(i);theta(i)];
        vertices = SE2Algebra.GetPointsInGlobalFrame(pushobj.shape_vertices, obj_pose);
        vertices(:,end+1) = vertices(:,1);
        if i == 1
            c= 'k';
        elseif i == num_rec_configs
            c = 'b';
        else
            c = 'r';
        end
        plot(vertices(1,:), vertices(2,:), '-', 'Color', c);
         R = [cos(theta(i)), -sin(theta(i)); sin(theta(i)), cos(theta(i))];
         hand_pt= R * hand_local_pt + [x(i);y(i)];
         drawCircle(hand_pt(1), hand_pt(2), hand_single_finger.finger_radius, 'k');
         hold on;
         plot(hand_pt(1), hand_pt(2), 'k-');
    end
end
axis equal;