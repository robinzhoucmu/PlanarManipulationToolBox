% Input: 
% obj_start_pos, obj_end_pos: 
function [h] = check2dtraj_visualize(vd_file, obj_start_pos, obj_end_pos, finger_2d_traj)
% Specify input would be in mm.
unit_scale = 1000;
H_tf = eye(4,4);
% Linear interpolation.
N = size(finger_2d_traj,1);
[ obj_2d_traj ] = LinearInterpObjPos(obj_start_pos, obj_end_pos, N, unit_scale, H_tf);
h = plot2dtraj(vd_file, obj_2d_traj, finger_2d_traj, unit_scale);
end

