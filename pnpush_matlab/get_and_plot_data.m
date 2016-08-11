function [object_pose, tip_pose, wrench] = get_and_plot_data(filename, shape_id, do_plot)
% Input: 
% 1. File path 
% 2. Shape used (e.g. 'rect1')
% 3. Boolean indicating if a plot of the experiment is desired
%
% Output: 
% 1. Position of the object w.r.t. the robot frame (time, x, y,
% orientation)
% 2. Position of the pusher tip w.r.t. the robot frame (tim, x, y)
% 3. WrenchL force in the x and y directions and torque.
%
% Example of how to run this function: 
% [object_pose, tip_pose, wrench] = get_and_plot_data( strcat(pwd, ...
% '/motion_surface=plywood_shape=hex_a=0_v=150_i=1.000_s=0.000_t=-0.349.json'), ...
% 'hex', 1);
%

addpath(strcat(pwd,'/Json/fsroot/jsonlab'));

data = loadjson(filename);
object_pose = data.object_pose; 
tip_pose = data.tip_pose;
wrench = data.ft_wrench;

if do_plot
    
    figure('position', [275, 500, 1500, 1000]) 
    
    %Plot snapshots object and pusher pose
    subplot(1,2,1); hold on;
    tip_radius = 0.00475;
    gamma = linspace(0,2*pi);
    plot_interval = 20;
    for i = 1:plot_interval:length(tip_pose(:,1))
         plot(tip_radius*cos(gamma)+tip_pose(i,2),tip_radius*sin(gamma)+tip_pose(i,3), 'k'); 
    end
    shape = get_shape(shape_id); 
    for i = 1:plot_interval:length(object_pose(:,1))
        theta = object_pose(i,4);
        rotation_matrix = [cos(theta) sin(theta); -sin(theta) cos(theta)];
        oriented_shape = shape*0;
        for j = 1:size(shape,1)            
            oriented_shape(j,:) = shape(j,:)*rotation_matrix;
        end
        plot(oriented_shape(:,1)+object_pose(i,2), oriented_shape(:,2) + object_pose(i,3), 'r');
    end
    title('Motion of the object and the pusher in the robot frame')
    %Plot force over time
    subplot(1,2,2); hold on;

    plot(wrench(:,1)-wrench(1,1), wrench(:,2), 'b');
    plot(wrench(:,1)-wrench(1,1), wrench(:,3), 'g');
    plot(wrench(:,1)-wrench(1,1), wrench(:,4), 'r');
    legend('x direction', 'y direction', 'torque')
    title('Force profile over time')
end
end