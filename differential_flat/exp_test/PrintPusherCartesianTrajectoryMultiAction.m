% Print out the trajectory x,y,z,qw,qx,qy,qz, action_id to execute on the robot.
% Input: traj_pusherframe (3*N), action_id(N*1) . The two are from PlanningGraph class.
% all input are in meters.
function [] = PrintPusherCartesianTrajectoryMultiAction(traj_pusherframe, action_id, table_z_height, csv_file_path)
num_waypoints = size(traj_pusherframe, 2);
traj_execute = zeros(num_waypoints, 7);
unit = 1000;
traj_execute(:, 1:2) = unit * traj_pusherframe(1:2, :)';
traj_execute(:, 3) = unit * ones(num_waypoints, 1) * table_z_height;
thetas = bsxfun(@plus, pi/2, traj_pusherframe(3, :));
traj_execute(:,4) = cos(thetas/ 2)';
traj_execute(:, 5:6) = zeros(num_waypoints, 2);
traj_execute(:, 7) = sin(thetas / 2)';
traj_execute(:,8) = action_id;
% Print csv to file
csvwrite(csv_file_path, traj_execute);
end