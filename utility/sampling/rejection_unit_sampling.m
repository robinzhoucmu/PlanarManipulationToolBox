function [x,y] = rejection_unit_sampling(num_samples)
sampled_data = rand(3 * num_samples, 2);
sampled_data = 2 * bsxfun(@minus, sampled_data, [0.5,0.5]);
filter = sqrt(sum(sampled_data.^2,2)) <= 1;
sampled_data = sampled_data(filter, :);
x = sampled_data(1:num_samples, 1);
y = sampled_data(1:num_samples, 2);
end