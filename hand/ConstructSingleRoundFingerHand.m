function [hand_single_finger] = ConstructSingleRoundFingerHand(finger_radius)
% For single finger, there is no offset between the hand and finger frames. 
fun_fk_hand_single_finger = @(q)([0;0;0]); 
fun_fv_hand_single_finger = @(q, qdot)([0;0;0]);

hand_single_finger = Hand();
hand_single_finger.finger_type = 'all_circles';
hand_single_finger.num_fingers = 1;
hand_single_finger.finger_radius = finger_radius;
hand_single_finger.fun_fk = fun_fk_hand_single_finger;
hand_single_finger.fun_fv = fun_fv_hand_single_finger;
hand_single_finger.finger_geometries{1} = [0;0];

end