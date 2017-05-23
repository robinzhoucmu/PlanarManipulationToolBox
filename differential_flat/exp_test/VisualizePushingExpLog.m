function [] = VisualizePushingExpLog(file_name, pushobj, hand, finger_width)
D = csvread(file_name);
filter = (abs(D(:,2)) < 1) & (abs(D(:,3)) < 1) & (abs(D(:,4)) < 0.01);
D(filter, :) = [];
t = D(:,1);
num_stamps = length(t);
unit = 1000;
obj_poses = D(:,2:4);
obj_poses(:, 1:2) = obj_poses(:, 1:2) / unit;
pusher_poses = D(:,5:7);
pusher_poses(:, 1:2) = pusher_poses(:, 1:2) / unit; 
action_ids = D(:,8);
seg_draw = floor(num_stamps / 30);
figure;
color_map = ['b', 'm', 'c'];
num_actions = 1;
for i = 1:1:num_stamps - 1
    flag_draw = (mod(i, seg_draw) == 1) | (action_ids(i+1) ~= action_ids(i)) | (i == (num_stamps - 1));
    if flag_draw
        % Draw the push object
        if (action_ids(i+1) ~= action_ids(i))
            c = 'k';
            num_actions = num_actions + 1;
            obj_poses(i,:)
            obj_poses(i+1,:)
        else
            c = color_map(num_actions);
        end
        vertices = SE2Algebra.GetPointsInGlobalFrame(pushobj.shape_vertices, obj_poses(i,:)');
        vertices(:,end+1) = vertices(:,1);
        plot(vertices(1,:), vertices(2,:), '-', 'Color', c);
        hold on;
    end
end
axis equal;
end