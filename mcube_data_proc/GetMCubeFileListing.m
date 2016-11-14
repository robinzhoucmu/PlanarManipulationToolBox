% folder_name: the json folder that contains all the related json files
% collected my MIT MCube lab. 
% For detailed description of the dataset: https://mcube.mit.edu/push-dataset
% query_info: specify multiple fields.
% 1) surface: 'plywood', ''
% 2) shape: 'rect1', 'butter', 'rect2', etc.
% 3) velocity: '10',
% It retrieves concatenated wrench and twist data from the jscon files that
% match the query_info.

% Sample usage: folder_name = '~/pushing_data'; query_info.surface='abs'; 
% query_info.shape = 'rect1'; query_info.velocity = 10;
function [listing] = GetMCubeFileListing(folder_name, query_info) 
str_folder = strcat(folder_name, '/', query_info.surface, '/', query_info.shape, '/');
% example: 'motion_surface=plywood_shape=rect1_a=0_v=10_i=0.000_s=0.000_t=0.349.json'
str_file = strcat('motion_surface=', query_info.surface, '_shape=', query_info.shape, ...
                  '*v=', num2str(query_info.velocity), '_i*.json');
listing = dir(strcat(str_folder, str_file));    
% Remove all near-empty files.
for i = 1:1:length(listing)
    if listing(i).bytes < 100
        delete(strcat(str_folder,listing(i).name));
    end
end
addpath(str_folder);
listing = dir(strcat(str_folder, str_file));    
end