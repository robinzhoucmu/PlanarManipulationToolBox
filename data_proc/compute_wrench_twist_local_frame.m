% Input:
% shape as vertices of polynogal object.
% obj_pose: N*3.
% force, tip_pt: N*2. contact point. All in global frame. Note that wrench
% is the force/torque applied by the object on the table.
% Output:
% return wrenches and twist in local robot frame.
function [wrench_local, twist_local] = compute_wrench_twist_local_frame(shape, force, obj_pose, tip_pt, t)
n = size(force, 1);
wrench_local = zeros(n-1, 3);
twist_local = zeros(n-1, 3);
for i = 1:1:n-1
    theta = obj_pose(i,3);
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    wrench_local(i,1:2) = (R' * force(i, 1:2)')';
    % Get closest point from the center of the cylindrical tip to the
    % object.
    cur_shape = bsxfun(@plus, obj_pose(i,1:2),(R * shape')');
    [tip_proj, dist] = projPointOnPolygon(tip_pt(i,:), cur_shape);
    % Contacting point in fixed robot based frame.
    tip_pt_contact = polygonPoint(cur_shape, tip_proj);
    tip_pt_local = R' * (tip_pt_contact - obj_pose(i,1:2))'; 
    %torque = pc_x * fy - pc_y * fx;
    wrench_local(i,3) = ...
        tip_pt_local(1) * wrench_local(i,2) - tip_pt_local(2) * wrench_local(i,1); 
    twist_local(i,1:2) = (R' * (obj_pose(i+1,1:2) - obj_pose(i,1:2))') / (t(i+1) - t(i));
    twist_local(i,3) = (obj_pose(i+1,3) - obj_pose(i,3)) / (t(i+1) - t(i));
end
% Change wrench to load by negating the direction.
wrench_local = -wrench_local;
end