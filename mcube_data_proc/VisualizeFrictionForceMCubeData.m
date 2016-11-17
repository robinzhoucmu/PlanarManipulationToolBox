function [h1,h2, h3] = VisualizeFrictionForceMCubeData(folder_name, surface_type, shape_id, vel, num_samples_perfile)
query_info.surface= surface_type; 
query_info.shape = shape_id; 
query_info.velocity = vel; 
if (nargin < 5)
    num_samples_perfile = 20;
end
file_listing = GetMCubeFileListing(folder_name, query_info);
[all_wrenches_local, all_twists_local, vel_tip_local, dists, vel_slip] = read_json_files(file_listing, shape_id, num_samples_perfile); 
% If the slipping component is less than 5% of the pushing velocity, then
% we consider it as sticking.
vel_sticking_threshold = (query_info.velocity/1000) * 0.05;
index_stick = abs(vel_slip) < vel_sticking_threshold;
h1 = figure;
figure(h1);
plot(all_wrenches_local(index_stick, 1), all_wrenches_local(index_stick, 2), 'b.');
str_title = strcat('Fxysticking-', shape_id, surface_type, num2str(vel));
title(str_title);
axis equal;
file_name = strcat(str_title, '.png');
saveas(h1, file_name);

h2 = figure;
figure(h2);
plot(all_wrenches_local(~index_stick, 1), all_wrenches_local(~index_stick, 2), 'r.');
str_title = strcat('Fxyslipping-', shape_id, surface_type, num2str(vel));
title(str_title);
axis equal;
file_name = strcat(str_title, '.png');
saveas(h2, file_name);

h3 = figure;
figure(h3);
str_title = strcat('Wrenches', shape_id, surface_type, num2str(vel));
plot3(all_wrenches_local(:,1), all_wrenches_local(:,2), all_wrenches_local(:,3), 'r.');
title(str_title);
axis equal;
file_name = strcat(str_title, '.fig');
saveas(h3, file_name);

end