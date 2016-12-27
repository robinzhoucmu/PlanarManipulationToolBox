% Matlab code for parsing sensor log output from ROS experiment.
function [pre_push_poses, post_push_poses, ft_readings, robot_pose_readings] = ParseLog(file_name)
    fid = fopen(file_name);
    ind = 1;
    num_pushes = fscanf(fid, '%d', 1);
    while ind <= num_pushes
        % Read pre_push_pose.
        pre_push_poses(1:7, ind) = fscanf(fid, '%f %f %f %f %f %f %f', [7 1]);
        % Read pos_push_pose;
        post_push_poses(1:7, ind) = fscanf(fid, '%f %f %f %f %f %f %f', [7 1]);
        % Read the number of force readings.
        num_ft_readings = fscanf(fid, '%d', 1);
        % Read ft sensor log.
        ft_readings{ind} = fscanf(fid, '%f %f %f', [3 num_ft_readings]);
        % Read the number of robot pose readings.
        num_robot_pose_readings = fscanf(fid, '%d', 1);
        robot_pose_readings{ind} = fscanf(fid, '%f %f %f %f %f %f %f %f', [8 num_robot_pose_readings]);
        ind = ind + 1;
    end
    fclose(fid);
end
