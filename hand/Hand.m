classdef Hand < handle
    % Hand class, contains hand kinematics and finger geometry.
    % We assume each finger is either a round circle or a polygon. 
    properties
        % Cell array of fingers storing the geometries. 
        fingers
        num_fingers
        % The first 3 elements of q are cartesian pose w.r.t global frame.
        % The rest are configurations of the fingers.
        q
        qdot
        % function pointer of forward kinematic in the local frame. 
        % Input of the hand configuration, return each of the finger pose.
        fun_fk
        % function pointer of returning the twists of fingers given q and qdot.
        fun_fv
    end
    
    methods
        function [obj] = SetQandQdot(obj, q, qdot)
            obj.q = q;
            obj.qdot = qdot;
        end
        % To call the function below, we must have set q and qdot.
        % Return the cartesian pose of each finger frame w.r.t the hand
        % frame.
        function [finger_carts] = GetFingerCartesiansWrtHand(obj)
            finger_carts = obj.fun_fk(obj.q);
        end
        
        % Output: 3 * num_fingers of finger cartesian pose in global frame.
        function [finger_carts_global] = GetGlobalFingerPositions(obj)
            finger_carts_wrt_hand = obj.GetFingerCartesiansWrtHand();
            finger_carts_global = zeros(3, obj.num_fingers);
            for i = 1:1:obj.num_fingers
                finger_carts_global(:, i) = SE2Algebra.GetCartPoseABCChain(obj.q(1:3), finger_carts_wrt_hand); 
            end
        end

        % Return the twists of each finger w.r.t the hand frame viewed from finger's own body frame.
        function [finger_twists] = GetFingerBodyTwistsWrtHand(obj)
            finger_twists = obj.fun_fv(obj.q, obj.qdot);
        end
        
        % Return the twists of each finger w.r.t the global inertia frame viewed from finger's own body frame.
        function [finger_twists] = GetFingerBodyTwistsWrtInertiaFrame(obj)
            finger_twists_wrt_hand_body = obj.GetFingerBodyTwistsWrtHand();
            hand_twists_wrt_inertia_body = obj.GetHandBodyTwistsWrtInertiaFrame();
            finger_carts = obj.GetFingerCartesiansWrtHand();
            finger_twists = SE2Algebra.GetBodyTwistABCChain(hand_twists_wrt_inertia_body, finger_twists_wrt_hand_body, finger_carts);
        end
        
        function [hand_twists] = GetHandBodyTwistsWrtInertiaFrame(obj)
            
        end
    end
    
end

