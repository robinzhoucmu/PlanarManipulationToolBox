function [pho] = compute_shape_avgdist_to_center(shape_id)
shape = get_shape(shape_id);
shape_vertices = shape(1:end-1, :);
shape_center = mean(shape_vertices);
offsets = bsxfun(@minus, shape_vertices, shape_center);
dists = sqrt(sum(offsets.^2, 2));
pho = mean(dists);
end