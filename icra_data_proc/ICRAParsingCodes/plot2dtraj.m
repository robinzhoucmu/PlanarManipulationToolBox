% pos_2d, pos_finger: row matrix.
function [h] = plot2dtraj(vd_file, pos_2d, pos_finger, unit)
if (nargin == 3)
    unit = 1000;
end
h = figure;
axis_xmin = 100; 
axis_xmax = 800;
axis_ymin = -200;
axis_ymax = 200;
axis([axis_xmin axis_xmax axis_ymin axis_ymax]);
tri_edge_length = 150;
pos_2d(:,1:2) = pos_2d(:,1:2) * unit;
pos_finger(:,1:2) = pos_finger(:,1:2) * unit; 
plot(pos_2d(:,1), pos_2d(:,2), 'r-');
%hold on;
movieObj = VideoWriter(vd_file);
movieObj.FrameRate = 10;
open(movieObj);
for i = 1:1:size(pos_2d,1)
    figure(h);
    %h = figure;
    hold on;
    axis equal;
    axis([axis_xmin axis_xmax axis_ymin axis_ymax]);
    theta = pos_2d(i,3);
    R = [cos(theta), -sin(theta); ...
         sin(theta), cos(theta)];
    x_axis_end = pos_2d(i,1:2)' + tri_edge_length * R(:,1);
    y_axis_end = pos_2d(i,1:2)' + tri_edge_length * R(:,2);
    % Plot object.
    plot([pos_2d(i,1), x_axis_end(1)], [pos_2d(i,2), x_axis_end(2)], 'r-', 'LineWidth', 2);
    plot([pos_2d(i,1), y_axis_end(1)], [pos_2d(i,2), y_axis_end(2)], 'g-', 'LineWidth', 2);
    plot([x_axis_end(1), y_axis_end(1)], [x_axis_end(2), y_axis_end(2)], 'b-', 'LineWidth', 2);
%     plot([pos_2d(i,1), x_axis_end(1)], [pos_2d(i,2), x_axis_end(2)], 'g*', 'MarkerSize', 10);
%     plot([pos_2d(i,1), y_axis_end(1)], [pos_2d(i,2), y_axis_end(2)], 'b*', 'MarkerSize', 10);
%     plot([x_axis_end(1), y_axis_end(1)], [x_axis_end(2), y_axis_end(2)], 'y*', 'MarkerSize', 10);
%     
    % Plot robot.
    plot(pos_finger(i, 1), pos_finger(i, 2), 'r*', 'MarkerSize', 6);
    frame = getframe(gcf);
    writeVideo(movieObj, frame);
    clf;
    hold off;
end
close(movieObj);
end

