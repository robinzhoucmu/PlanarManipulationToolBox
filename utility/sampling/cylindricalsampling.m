% Given the radius and height of the cylinder, sample uniformly in the
% cylindrical space. In each z-height plane (a total of num_samples_planes
% in [-h/2, h/2]), polar coordinate based sampling (sun flower sampling)   
   function [S, x,y,theta] = cylindricalsampling(r, h, num_samples_circle, num_samples_plane, ratio_complete_random)
if nargin < 5
   ratio_complete_random = 0;
end
theta = linspace(-h/2, h/2, num_samples_plane + 1)';
theta = theta(2:end);
% [x, y] = sunflower_sampling(num_samples_circle, 2);
% x = r * x;
% y = r * y;
% S = zeros(3, num_samples_circle * num_samples_plane);
% for i = 1:1:num_samples_plane
%     S(:, (i-1) * num_samples_circle + 1: i*num_samples_circle) = ...
%         [x';y';theta(i) * ones(1,num_samples_circle)];
% end
S = zeros(3, num_samples_circle * num_samples_plane);
for i = 1:1:num_samples_plane
    [x,y] = rejection_unit_sampling(num_samples_circle);
    x = r*x;
    y = r*y;
    S(:, (i-1) * num_samples_circle + 1: i*num_samples_circle) = ...
        [x';y';theta(i) * ones(1,num_samples_circle)];
end
% Random shuffling.
rand_perm_indices = randperm(size(S, 2));
S = S(:, rand_perm_indices);
% Now for the specified ratio_complete_random, we randomly generate thetas.
num_complete_random = size(S,2) * ratio_complete_random;
thetas_random = 2 * (rand([1 num_complete_random]) - 0.5) * h/2;
S(3, 1:num_complete_random) = thetas_random;
end
