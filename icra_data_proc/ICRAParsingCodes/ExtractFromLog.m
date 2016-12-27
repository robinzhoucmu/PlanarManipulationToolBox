function [ record_log ] = ExtractFromLog( log_file_name, pho, R_tool, H_tf, unit_scale)
[pre_push_poses, post_push_poses, ft_readings, robot_pose_readings] = ParseLog(log_file_name);
num_pushes = size(pre_push_poses, 2);

record_log.num_pushes = num_pushes;
record_log.pre_push_poses = pre_push_poses;
record_log.post_push_poses = post_push_poses;

push_wrenches = zeros(num_pushes, 3);
slider_velocities = zeros(num_pushes, 3);
slider_vel_raw = zeros(num_pushes, 3);
disp_raw = zeros(num_pushes, 3);
for i = 1:1:num_pushes
    % Get object pre and post push pose.
    obj_start_pos = pre_push_poses(:,i);
    obj_end_pos = post_push_poses(:,i);
    % Use force reading time as timeline.
    t = ft_readings{i}(1, :);
    t0 = t(1);
    t = bsxfun(@minus, t, t0);
    force = ft_readings{i}(2:3, :);
    % Force torque sensor positive x is in the opposite direction of global
    % robot frame.
    force = R_tool * force;
    % Negate sign to get force applied on the object.
    force = - force;
    % Minus offset. 
    force = bsxfun(@minus, force, force(:,1));
    record_log.raw_force{i}= force;
    %Eliminate forces before the jump of the signal.
    avg_f = mean(force,2);
    index_small = sum(force.^2,1) < sum(avg_f.^2);
    
    % Hack: only use x%-100% of the force signal.
    starting_p = 0.3;
    index_pre_touch = 1:length(t) < ceil(length(t) * starting_p);
    index_rm = index_pre_touch | index_small;
    force(:, index_rm) = [];
    t(index_rm) = [];    
    N = length(t); 
    record_log.filtered_force{i} = force;
    % Interpolate (linear) object poses. 
    [ obj_2d_traj ] = LinearInterpObjPos(obj_start_pos', obj_end_pos', N, unit_scale, H_tf);
    record_log.obj_2d_traj{i} = obj_2d_traj;
    
    % Get robot trajectory. 
    robot_traj_0 = robot_pose_readings{i}(2:end, :);
    t_robot = robot_pose_readings{i}(1,:);
    t_robot = bsxfun(@minus, t_robot, t0);
   
    % Linear interpolate robot_pose.
    robot_traj = interp1(t_robot,robot_traj_0',t, 'linear','extrap')';
    robot_2d_pos = get2dPos(robot_traj',eye(4,4), unit_scale);
    record_log.robot_traj{i} = robot_traj;
    record_log.robot_2d_pos{i} = robot_2d_pos;
    
    % Compute wrench in object local coordinate.
    wrench = ComputeWrench(obj_2d_traj, force', robot_2d_pos);
    % Minus offset. 
    wrench(:,3) = wrench(:,3) / pho;
    record_log.wrench{i} = wrench;  
    % Trajectory video generation.
    %vd_file = strcat('videos/out', int2str(i), '.avi');
    %finger_2d_traj = robot_traj(1:2,:)'; [h] = check2dtraj_visualize(vd_file, obj_start_pos', obj_end_pos', finger_2d_traj/unit_scale);
    
    % Compute average wrench as the pushing force and normalized slider velocity. 
    push_wrenches(i,:) = mean(wrench);
    d = obj_2d_traj(end,:) - obj_2d_traj(1,:);
    % Rotate to initial object frame.
    theta = obj_2d_traj(1,3);
    R = [cos(theta), -sin(theta); ...
         sin(theta), cos(theta)];
    d(1:2) = (R' * d(1:2)')';
    disp_raw(i,:) = d;
    slider_vel_raw(i,:) = d / norm(d);
    d(3) = d(3) * pho;
    slider_velocities(i,:) = d / norm(d);
end
record_log.push_wrenches = push_wrenches;
record_log.slider_velocities = slider_velocities;
record_log.disp_raw = disp_raw;
end

