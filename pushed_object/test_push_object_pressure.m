rect_shape =  get_shape('rect2');
vertices = rect_shape(1:end-1,:);

options_support_pts.mode = 'polygon';
options_support_pts.vertices = vertices;

num_supports_pts = 100; 
support_pts = GridSupportPoint(num_supports_pts, options_support_pts); % N*2.

options_pressure.mode = 'uniform';
pressure_weights = AssignPressure(support_pts, options_pressure);

% Compute center of friction. (2*1)
cof = support_pts' * (pressure_weights / sum(pressure_weights));

% Translate local coordinate frame such that the COF is the origin.
support_pts = bsxfun(@minus, support_pts, cof');

shape_info.shape_id = 'rect2';
shape_info.shape_type = 'polygon';
shape_info.shape_vertices = vertices';
shape_info.pho = mean(sqrt(sum(shape_info.shape_vertices.^2)));
rect_pushobj = PushedObject(support_pts', pressure_weights, shape_info)