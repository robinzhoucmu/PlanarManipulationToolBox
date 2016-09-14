function [all_results, simulation_inst] = ComputeThreeFingersCircleCaptureRegionPitchFunction(pushobj, pitch_fun, mu, ratio_uncertainty, finger_radius, num_init_samples, flag_stop_first_contact)
if (nargin < 7)
flag_stop_first_contact = 0;
end
rng(1);
uncertainty_radius = pushobj.shape_parameters.radius * ratio_uncertainty;

max_quasistatic_vel = pushobj.shape_parameters.radius; % one radius per second.
finger_movement_sample_dt = 0.01;

eps_init_dist = 0.001;
finger_traj = PitchCompute(pitch_fun, ... 
                           max_quasistatic_vel, ...
                           pushobj.shape_parameters.radius, ...
                           uncertainty_radius + finger_radius + eps_init_dist, ...
                           finger_movement_sample_dt);

finger_traj.plot(3);

simulation_inst = SimulationWorld(pushobj, finger_traj, mu, finger_radius, 3);
ct_samples = 0;

% % Uniform grid sampling.
% R = pushobj.shape_parameters.radius;
% num_samples = num_init_samples * (4 / pi);
% grid_size = sqrt(4*R*R / num_samples);
% % Form the grids.
% Px(:,1) = -R:grid_size:R;
% Py(:,2) = -R:grid_size:R;
% 
% [X, Y] = meshgrid(Px(:,1), Py(:,2));
% P(:,1) = X(:);
% P(:,2) = Y(:);
% % Select grids inside.
% InCircle = sum(P.^2,2) <= R^2;
% Pts = P(InCircle, :);


[rx, ry] = sunflower(num_init_samples, 2);
Pts = pushobj.shape_parameters.radius * [rx,ry];


num_init_samples = size(Pts, 1);
all_results = cell(num_init_samples, 1);
while (ct_samples < num_init_samples)
    (ct_samples + 0.0) / num_init_samples
    % Sample a random initial pose such that the object circle center is
    % within (ratio_uncertainty - 1) * circle_radius.
    %sampling_radius = (uncertainty_radius - pushobj.shape_parameters.radius);
    %init_pose = 2 * bsxfun(@minus, rand(2,1), 0.5) * sampling_radius;

    % Rejection sampling.
    %if norm(init_pose) < sampling_radius
        ct_samples = ct_samples + 1;
        init_pose = [Pts(ct_samples, :)';0];
        % Set initial pose of the object.
        simulation_inst.pushobj.pose = init_pose;
        flag_plot = 1;
        [flags, pose_log, twist, center_linear_vel] = simulation_inst.SimulationRollOut(flag_plot, flag_stop_first_contact);
        all_results{ct_samples}.init_pose = init_pose;
        all_results{ct_samples}.result_flags = flags;
        all_results{ct_samples}.pose_log = pose_log;
        all_results{ct_samples}.twist = twist;
        all_results{ct_samples}.center_linear_vel = center_linear_vel;
    %end
end

% Plot capture regions from result logs.
figure;
hold on;
num_grasped = 0;
num_jammed = 0;
num_missed = 0;
for i = 1:1:ct_samples
    x = all_results{i}.init_pose(1);
    y = all_results{i}.init_pose(2);
    if (all_results{i}.result_flags.grasped == 1)
        plot(x, y, 'r*');
        num_grasped = num_grasped + 1;
    elseif (all_results{i}.result_flags.jammed == 1)
        plot(x, y, 'k.');
        num_jammed = num_jammed + 1;
    else % missed.
        plot(x, y, 'b.');
        if (flag_stop_first_contact)
            length_ratio = 0.1;
            %plot([x, x + length_ratio * all_results{i}.center_linear_vel(1, end)], ...
            %    [y, y + length_ratio * all_results{i}.center_linear_vel(2, end)], 'r-');
            if (size(all_results{i}.center_linear_vel, 2) > 0)
                h = quiver(x,y, all_results{i}.center_linear_vel(1, end), all_results{i}.center_linear_vel(2, end), length_ratio, 'b-');
                set(h,'MaxHeadSize',200);
                set(h, 'LineWidth', 1.75);
            end
        end
        num_missed = num_missed + 1;
    end
end
axis equal;
num_grasped, num_jammed, num_missed
end

function [x,y] = sunflower(n, alpha)   %  example: n=500, alpha=2
    x = zeros(n,1);
    y = zeros(n,1);
    b = round(alpha*sqrt(n));      % number of boundary points
    phi = (sqrt(5)+1)/2;           % golden ratio
    for k=1:n
        r = radius(k,n,b);
        theta = 2*pi*k/phi^2;
        x(k) = r*cos(theta);
        y(k) = r*sin(theta);
    end
end

function r = radius(k,n,b)
    if k>n-b
        r = 1;            % put on the boundary
    else
        r = sqrt(k-1/2)/sqrt(n-(b+1)/2);     % apply square root
    end
end