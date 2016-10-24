%% Single round finger hand.
% fun_fk_hand_single_finger = @(q)([0;0;0]); 
% fun_fv_hand_single_finger = @(q, qdot)([0;0;0]);
% 
% hand_single_finger = Hand();
% hand_single_finger.num_fingers = 1;
% hand_single_finger.finger_radius = 0.005;
% hand_single_finger.fun_fk = fun_fk_hand_single_finger;
% hand_single_finger.fun_fv = fun_fv_hand_single_finger;
% 
% q0 = [0.1;0.1;pi/4];
% qdot0 = [0.05;-0.05; pi/6];
% hand_single_finger.SetQandQdot(q0, qdot0);
% hand_bodytwists_wrt_inertia_body = hand_single_finger.GetHandBodyTwistWrtInertiaFrame()
% hand_globaltwists_wrt_inertia_body = hand_single_finger.GetHandGlobalTwistWrtInertiaFrame()
% 
% 
% finger_cart = hand_single_finger.GetGlobalFingerCartesians()
% finger_bodytwist_wrt_hand = hand_single_finger.GetFingerBodyTwistsWrtHand()
% finger_bodytwist_wrt_inertiaframe = hand_single_finger.GetFingerBodyTwistsWrtInertiaFrame()
% 
% [finger_globaltwists_wrt_inertialframe, finger_carts_wrt_inertialframe] = ...
%     hand_single_finger.GetFingerGlobalTwistsAndCartesianWrtInertiaFrame()

%% Two round finger parallel gripper.
% Q is of 4 dof. q4 is the distance between the two fingers. 
fun_fk_hand_two_fingers = @(q)([0, 0; q(4)/2, -q(4)/2; 0, 0]);
fun_fv_hand_two_fingers = @(q,qdot)([0,0; qdot(4)/2, -qdot(4)/2; 0, 0]);

hand_two_fingers_gripper = Hand();
hand_two_fingers_gripper.num_fingers = 2;
hand_two_fingers_gripper.finger_radius = 0.005;
hand_two_fingers_gripper.fun_fk = fun_fk_hand_two_fingers;
hand_two_fingers_gripper.fun_fv = fun_fv_hand_two_fingers;

q0 = [0;0;pi/4;0.1];
qdot0 = [0.05;0.05;0;-0.1];
hand_two_fingers_gripper.SetQandQdot(q0, qdot0);
hand_bodytwists_wrt_inertia_body = hand_two_fingers_gripper.GetHandBodyTwistWrtInertiaFrame()
hand_globaltwists_wrt_inertia_body = hand_two_fingers_gripper.GetHandGlobalTwistWrtInertiaFrame()


finger_cart = hand_two_fingers_gripper.GetGlobalFingerCartesians()
finger_bodytwist_wrt_hand = hand_two_fingers_gripper.GetFingerBodyTwistsWrtHand()
finger_bodytwist_wrt_inertiaframe = hand_two_fingers_gripper.GetFingerBodyTwistsWrtInertiaFrame()

[finger_globaltwist_wrt_inertiaframe, finger_carts_wrt_inertialframe] = ...
    hand_two_fingers_gripper.GetFingerGlobalTwistsAndCartesianWrtInertiaFrame()

finger_linear_globalvels = zeros(3,2);
for i = 1:1:2
    finger_linear_globalvels(:,i) = SE2Algebra.GetTwistMatrix(finger_globaltwist_wrt_inertiaframe(:,i)) * [finger_cart(1:2,i);1];
end
finger_linear_globalvels = finger_linear_globalvels(1:2,:)

finger_linear_bodylvels = zeros(3,2);
for i = 1:1:2
    finger_linear_bodyvels(:,i) = SE2Algebra.GetTwistMatrix(finger_bodytwist_wrt_inertiaframe(:,i)) * [0;0;1];
end
finger_linear_bodyvels = finger_linear_bodyvels(1:2,:)

