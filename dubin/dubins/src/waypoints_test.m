clc
clear all
close all

compileMex = 1;
if compileMex
    mex dubins.cpp dubins_matlab.cpp -I../include
end

%stepSize = .1; % Step size for calculating points
num_steps = 500;
rho = 5; % Turning radius

q0 = [-rho, 0    , 0];
q1 = [20  , 2*rho, pi];
q2 = [0   , 4*rho, 0];
q3 = [20  , 6*rho, pi];
q4 = [-rho, 6*rho, pi];

waypoints = [q0;q1;q2;q3;q4;q0];

figure
hold on
for i=1:size(waypoints, 1)-1
    path = dubins(waypoints(i,:), waypoints(i+1,:), rho, num_steps);
    x = path(1,:);
    y = path(2,:);
    theta = path(3,:);
    d = path(4,:);
    scatter(x, y)
    %quiver(x, y, cos(theta), sin(theta),.8,'o');
end
axis equal
title('Waypoint navigation using Dubins path')
xlabel('x [m]'); ylabel('y [m]');
hold off
