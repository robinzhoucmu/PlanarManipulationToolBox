function [] = VisualizePushingExpLog(file_name, pushobj, hand)
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
color_map = ['b', 'm', 'r'];
num_actions = 1;
unit = 1;
for i = 1:1:num_stamps - 1
    flag_draw = (mod(i, seg_draw) == 1) || (action_ids(i+1) ~= action_ids(i)) || (i == (num_stamps - 1));
   vertices = SE2Algebra.GetPointsInGlobalFrame(pushobj.shape_vertices, obj_poses(i,:)');
   vertices(:,end+1) = vertices(:,1);
   vertices = vertices * unit;
    if flag_draw
        % Draw the push object
        if ( i == 1 || i == num_stamps - 1)
            linewidth = 2.0;
            linestyle = '-';
        end
        
        if (action_ids(i+1) ~= action_ids(i))
            c = 'k';
            num_actions = num_actions + 1;
            linewidth = 2.0;
            linestyle = '-';
        else
            c = color_map(num_actions);
            linewidth = 1.5;
            linestyle = '--';
        end

        %plot(vertices(1,:), vertices(2,:),  'Color', c, 'LineStyle', linestyle, 'LineWidth', linewidth);
        fill(vertices(1,:), vertices(2,:), c, 'FaceAlpha', 0.05 , 'EdgeColor', c, 'EdgeAlpha', 0.5);
        hold on;
        q = hand.q;
        q(1:3) = pusher_poses(i,:)' * unit;
        %q(4) = width_finger;
        hand.Draw(gcf, q, c, 2);
        hold on;
        if (( i == 1 || i == num_stamps - 1))
            fill(vertices(1,:), vertices(2,:), 'k', 'FaceAlpha', 0.5 , 'EdgeColor', 'k');
            hold on;
        end
    end
end
safe_buffer = 20;
table_size_x = (451.6 - 100) / 1000.0;
table_size_y = (254.0 - 10)/ 1000.0;
table_center = [0; -317.5/1000];
axis([-0.15, 0.15, table_center(2) - 0.08, table_center(2) + 0.08]);
axis equal;
end