% object_found: True
% pose: 
%   x: -0.950840393559
%   y: -316.747312616
%   theta: -0.0338589771565
% 
% The lower left corner of the block used for alignment, so the corner pose is 
% x - 0.05/2
% y - 0.035/2
% theta
% 
% object_found: True
% pose: 
%   x: -17.6984943658
%   y: -322.598002728
%   theta: -0.0262008026687
pose_corner = [-0.950840393559/1000; -316.747312616/1000; -0.0338589771565];
pose_corner = pose_corner - [0.05/2;0.035/2;0];
pose_com_wrt_corner = [0.0275/3;0.058/3;-pi/2];
pose_april = [-17.6984943658 / 1000; -322.598002728/1000; -0.0262008026687];

T_pose_corner = SE2Algebra.GetHomogTransfFromCartesianPose(pose_corner);
T_com_wrt_corner = SE2Algebra.GetHomogTransfFromCartesianPose(pose_com_wrt_corner);
T_pose_com = T_pose_corner * T_com_wrt_corner;

T_pose_april = SE2Algebra.GetHomogTransfFromCartesianPose(pose_april);
T_com_wrt_april = inv(T_pose_april) * T_pose_com;
pose_com_wrt_april = SE2Algebra.GetCartesianPoseFromHomogTransf(T_com_wrt_april)