function [hand_parallel_edges] = ConstructParallelEdgeGripper(virtual_circle_radius, finger_length, finger_width)
% Input: virtual circle radius (blem) enclosing the rectangle edge gripper. 
% the length of the edge gripper (the length two fingers facing each
% other). the width of edge (if not specified then we set it as 25% of the
% length).
if (nargin < 2)
    finger_width = 0.25 * finger_length;
end
% Q is of 4 dof. q4 is the distance between the two fingers. 
fun_fk_hand_two_fingers = @(q)([0, 0; q(4)/2, -q(4)/2; 0, 0]);
fun_fv_hand_two_fingers = @(q,qdot)([0,0; qdot(4)/2, -qdot(4)/2; 0, 0]);
% Set the finger frame center as the mid point of inward edges.
% Same x-y positive orientation. 
% CCW from the upper left point.
finger_shape_lower = ...
    [ -finger_length/2, -finger_length/2, finger_length/2, finger_length/2 ;
      0, -finger_width, -finger_width, 0 ];
% CCW from the lower left point.
finger_shape_upper = ...
    [-finger_length/2, finger_length/2, finger_length/2, -finger_length/2;
    0, 0, finger_width, finger_width];

hand_parallel_edges = Hand();
hand_parallel_edges.finger_radius = virtual_circle_radius;
hand_parallel_edges.num_fingers = 2;
hand_parallel_edges.finger_type = 'multi_polygons';
hand_parallel_edges.finger_geometries{1} = finger_shape_upper;
hand_parallel_edges.finger_geometries{2} = finger_shape_lower;
hand_parallel_edges.fun_fk = fun_fk_hand_two_fingers;
hand_parallel_edges.fun_fv = fun_fv_hand_two_fingers;

end