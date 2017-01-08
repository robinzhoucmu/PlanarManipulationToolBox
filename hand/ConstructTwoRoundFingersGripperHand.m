function [hand_two_fingers_gripper] = ConstructTwoRoundFingersGripperHand(finger_radius)
fun_fk_hand_two_fingers = @(q)([0, 0; q(4)/2, -q(4)/2; 0, 0]);
fun_fv_hand_two_fingers = @(q,qdot)([0,0; qdot(4)/2, -qdot(4)/2; 0, 0]);

hand_two_fingers_gripper = Hand();
hand_two_fingers_gripper.num_fingers = 2;
hand_two_fingers_gripper.finger_type = 'all_circles';
hand_two_fingers_gripper.finger_radius = finger_radius;
hand_two_fingers_gripper.finger_geometries{1} = [0;0];
hand_two_fingers_gripper.finger_geometries{2} = [0;0];

hand_two_fingers_gripper.fun_fk = fun_fk_hand_two_fingers;
hand_two_fingers_gripper.fun_fv = fun_fv_hand_two_fingers;    

end