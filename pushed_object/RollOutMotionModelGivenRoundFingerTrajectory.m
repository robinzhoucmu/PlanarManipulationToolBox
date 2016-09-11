% Input: push_obj with ls coefficients, type and initial pose specified. 
% tip_pt x,y position over t. 2*N.
% finger radius.
% Output: 
% obj_pose over time.
% All input, output poses are in world frame. Column vectors.
function [obj_pose] = RollOutMotionModelGivenRoundFingerTrajectory(pushobj, tip_pt, t, finger_radius)
N = length(t);
assert(size(tip_pt,2 ) == N);
obj_pose = zeros(3, N);
obj_pose(:,1) = pushobj.pose;
const_mu = 0.6;
plot_interval = ceil(N/5.0);
figure;
for i = 1:1:N - 1
    dt = t(i+1) - t(i);
    pt_finger_center = tip_pt(:, i);
    twist = [(tip_pt(:,i+1) - tip_pt(:, i)) / dt;0]
    theta = pushobj.pose(3);
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)]; 
    [flag_contact, pt_contact, vel_contact, outward_normal_contact] = ...
              pushobj.GetRoundFingerContactInfo(pt_finger_center, finger_radius, twist)
    if (flag_contact)
        [twist_local, wrench_load_local, contact_mode] = ...
            pushobj.ComputeVelGivenPointRoundFingerPush(pt_contact, vel_contact, outward_normal_contact, const_mu);
        % Convert local twist to global frame.
        pushobj.pose(1:2) = pushobj.pose(1:2) + R * twist_local(1:2) * dt;
        pushobj.pose(3) = pushobj.pose(3) + twist_local(3) * dt;
        %Adg = [R, [pushobj.pose(2);-pushobj.pose(1)];0,0,1];
        %twist_global = Adg * twist_local
        %pushobj.pose = pushobj.pose + twist_global * (t(i+1) - t(i))  
    end
    obj_pose(:,i+1) = pushobj.pose;
    if (mod(i, plot_interval) == 1)
        drawCircle(pt_finger_center(1), pt_finger_center(2), finger_radius, 'color', 'k');
        hold on;
        cur_shape = bsxfun(@plus, R * pushobj.shape_vertices, pushobj.pose(1:2));
        if i == 1
            polycolor = 'k'
        else
            polycolor = 'r'
        end
        drawPolygon(cur_shape', 'color', polycolor);
    end
end

end