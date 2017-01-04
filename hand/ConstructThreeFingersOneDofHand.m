function [hand_three_fingers_gripper] = ConstructThreeFingersOneDofHand(finger_radius)
% The configuration of the 1dof three finger gripper (mlab simple hand):
% x,y,theta of the hand center frame + d the distance of the finger to the
% center. The three fingers are separated by 120 degrees from each other.
% When theta = 0, the first finger is on the postive x-axis, q(4) distance
% away from the origin.
fun_fk_hand_three_fingers = @(q)([0, sqrt(3)/2*q(4), -sqrt(3)/2*q(4); q(4), -q(4)/2, -q(4)/2; 0, 0, 0]);
fun_fv_hand_three_fingers = @(q,qdot)([0, sqrt(3)/2*qdot(4), -sqrt(3)/2*qdot(4); qdot(4), -qdot(4)/2, -qdot(4)/2; 0, 0, 0]);

hand_three_fingers_gripper = Hand();
hand_three_fingers_gripper.finger_type = 'all_circles';
hand_three_fingers_gripper.num_fingers = 3;
hand_three_fingers_gripper.finger_radius = finger_radius;
hand_three_fingers_gripper.fun_fk = fun_fk_hand_three_fingers;
hand_three_fingers_gripper.fun_fv = fun_fv_hand_three_fingers;    
end