% Assume the pushing point is at (0, -r). The y axis of the local
% frame aligns the inward normal.
% Input: pose_start, pose_end, start and goal pose in world frame. 
% parameters.a : A_{11}, A_{12} (assume equal) for diagnal representation
% of limit surface. 
% parameters.b: A_{33} / rho^2.
% parameters.r: the distance from the pushing point to the object local
% frame origin (COM).
% parameters.mu (coefficient of friction) at the contact point.
% Output:
% x,y,theta: cartesian pose in world frame of the object. Length N. 
% u: pushing displacement segment in local frame. 2* N-1.
% z: output in flat space.
function [x, y, theta, u, z] = GetDubinPath(pose_start, pose_end, parameters, step_size)
a = parameters.a;
b = parameters.b;
r = parameters.r;
mu = parameters.mu;
radius_turn =  (a / (b* r * mu));
if nargin < 4
    step_size = radius_turn /50;
end
z_start = zeros(3,1);
z_end = zeros(3,1);
z_start(1:2) = pose_start(1:2) + a/(b*r)* [-sin(pose_start(3)); cos(pose_start(3))];
z_start(3) = pose_start(3) + pi/2;
z_end(1:2) = pose_end(1:2) + a/(b*r)* [-sin(pose_end(3)); cos(pose_end(3))];
z_end(3) = pose_end(3) + pi/2;
z = dubins(z_start', z_end', radius_turn, step_size);

T = 1.0;
num_z = size(z, 2);
dt = T / num_z;
[x,y,theta] = GetOrigStateFromFlatOutput(dt, z(1:2,:), a, b, r);
% Add the final goal pose.
x(end+1) = pose_end(1);
y(end+ 1) = pose_end(2);
theta(end + 1) = pose_end(3);

num_pts = length(x);
hand_q_traj = zeros(3, num_pts);
hand_q_traj(3,:) = theta;
hand_local_pt = [0; -r];
for i = 1:1:num_pts
    R = [cos(theta(i)), -sin(theta(i)); sin(theta(i)), cos(theta(i))];
    hand_q_traj(1:2, i) = R * hand_local_pt + [x(i);y(i)];
end
u = zeros(2, num_pts - 1);
for i = 1:1:num_pts - 1
    R = [cos(theta(i)), -sin(theta(i)); sin(theta(i)), cos(theta(i))];
    u(:, i) = R'*(hand_q_traj(1:2, i + 1) - hand_q_traj(1:2, i));
end
end