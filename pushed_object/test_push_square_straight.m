addpath('~/Downloads/cvx/');
cvx_startup;
shape_info.shape_id = 'square';
shape_info.shape_type = 'polygon';
edge_length = 0.02;
shape_info.shape_vertices = (edge_length/2)*[1,-1,-1,1;1,1,-1,-1];
shape_info.pho = edge_length;

options_support_pts.mode = 'polygon';
options_support_pts.vertices = shape_info.shape_vertices';

num_supports_pts = 100; 
support_pts = GridSupportPoint(num_supports_pts, options_support_pts); % N*2.

options_pressure.mode = 'uniform';
pressure_weights = AssignPressure(support_pts, options_pressure);


pushobj = PushedObject(support_pts', pressure_weights, shape_info, 'poly4')
pushobj.FitLS('poly4', 200, 0.1);

% Set the circle at the point of origin.
pushobj.pose = [0;0;0];

const_v_magnitude = 0.01;
const_v = const_v_magnitude * [0;-1];

const_mu = 0.2;
finger_radius = 0.002;
init_finger_pos = [-edge_length/8; 0.55*edge_length + finger_radius; 0];

total_time = 1.5;
total_num_t = 30;

t = linspace(0, total_time, total_num_t);
figure;
for i = 1:1:total_num_t-1
    dt = t(i+1) - t(i);
    theta = pushobj.pose(3);
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    pt_finger_center = init_finger_pos(1:2) + t(i) * const_v;
    %pushobj.pose
    drawCircle(pt_finger_center(1), pt_finger_center(2), finger_radius, 'color', 'k');
    hold on;
    cur_shape = bsxfun(@plus, R * pushobj.shape_vertices, pushobj.pose(1:2));
    drawPolygon(cur_shape', 'color', 'r');
    twist = [const_v;0];
    [flag_contact, pt_contact, vel_contact, outward_normal_contact] = ...
          pushobj.GetRoundFingerContactInfo(pt_finger_center, finger_radius, twist)
    if (flag_contact)
        [twist_local, wrench_load_local, contact_mode] = ...
            pushobj.ComputeVelGivenPointRoundFingerPush(pt_contact, vel_contact, outward_normal_contact, const_mu)
        % Convert local twist to global frame.
        pushobj.pose(1:2) = pushobj.pose(1:2) + R * twist_local(1:2) * dt;
        pushobj.pose(3) = pushobj.pose(3) + twist_local(3) * dt;
        %Adg = [R, [pushobj.pose(2);-pushobj.pose(1)];0,0,1];
        %twist_global = Adg * twist_local
        %pushobj.pose = pushobj.pose + twist_global * (t(i+1) - t(i))  
    end

    
end
axis equal;
