% Given object pre push and post push pose, generate 2d trajectory.
function [ obj_2d_traj ] = LinearInterpObjPos(obj_start_pos, obj_end_pos, N, unit_scale, H_tf)
if (nargin <= 3)
    unit_scale = 1000;
end
if (nargin <= 4)
    H_tf = eye(4,4);
end
obj_start_pos_2d = get2dPos(obj_start_pos, H_tf, unit_scale);
obj_end_pos_2d = get2dPos(obj_end_pos, H_tf, unit_scale);
obj_2d_traj = zeros(N,3);
for i = 1:1:3
    obj_2d_traj(:,i) = linspace(obj_start_pos_2d(i), obj_end_pos_2d(i), N);
end
%obj_2d_traj = obj_2d_traj';
end

