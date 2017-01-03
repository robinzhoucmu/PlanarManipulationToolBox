function [cur_shape_vertices] = GetPolygonShapeInWorldFrame(shape_vertices, pose)
    theta = pose(3);
    R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
    cur_shape_vertices = bsxfun(@plus, R * shape_vertices, pose(1:2));
end