% Single round finger hand.
fun_fk_hand_single_finger = @(q)([0;0;0]); 
fun_fv_hand_single_finger = @(q, qdot)([0;0;0]);

hand_single_finger = Hand();
hand_single_finger.num_fingers = 1;
hand_single_finger.finger_radius = 0.005;
hand_single_finger.fun_fk = fun_fk_hand_single_finger;
hand_single_finger.fun_fv = fun_fv_hand_single_finger;

q0 = [0.1;0.1;pi/4];
qdot0 = [0.05;-0.05; pi/6];
hand_single_finger.SetQandQdot(q0, qdot0);
hand_bodytwists_wrt_inertia_body = hand_single_finger.GetHandBodyTwistWrtInertiaFrame()
hand_globaltwists_wrt_inertia_body = hand_single_finger.GetHandGlobalTwistWrtInertiaFrame()


finger_cart = hand_single_finger.GetGlobalFingerCartesians()
finger_bodytwist_wrt_hand = hand_single_finger.GetFingerBodyTwistsWrtHand()
finger_bodytwist_wrt_inertiaframe = hand_single_finger.GetFingerBodyTwistsWrtInertiaFrame()

[finger_globaltwists_wrt_inertialframe, finger_carts_wrt_inertialframe] = ...
    hand_single_finger.GetFingerGlobalTwistsAndCartesianWrtInertiaFrame()
