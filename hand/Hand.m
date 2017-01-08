classdef Hand < handle
    % Hand class, contains hand kinematics and finger geometry.
    % We assume each finger is either a round circle or a polygon. 
    % Current implementation only supports round finger.
    properties
        % Finger types: 'all_circles', 'multi_polygons'.
        finger_type
        % Cell array of fingers storing the geometries if each
        % finger geometry is specified. 
        % For 'multi_polygons', each element is a 2*N array.
        finger_geometries
        % Finger radius for 'all_circle' type hand.
        % The enveloped virtual circle (blem model) radius for
        % 'multi_polygons' type.
        finger_radius
        num_fingers
        % The first 3 elements of q are cartesian pose w.r.t global frame.
        % The rest are configurations of the fingers.
        q
        qdot
        % function pointer of forward kinematic in the local frame. 
        % Input of the hand configuration, return each of the finger cartesian 
        % pose w.r.t the hand frame.
        fun_fk
        % function pointer of returning the body twists of fingers given q and qdot
        % with respect to the hand frame.
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
        function [finger_carts_global] = GetGlobalFingerCartesians(obj)
            finger_carts_wrt_hand = obj.GetFingerCartesiansWrtHand();
            finger_carts_global = zeros(3, obj.num_fingers);
            for i = 1:1:obj.num_fingers
                finger_carts_global(:, i) = SE2Algebra.GetCartPoseABCChain(obj.q(1:3), finger_carts_wrt_hand(:, i)); 
            end
        end

        % Return the twists (3 * num_fingers) of each finger w.r.t the hand frame viewed from finger's own body frame.
        function [finger_twists] = GetFingerBodyTwistsWrtHand(obj)
            finger_twists = obj.fun_fv(obj.q, obj.qdot);
        end
        
        % Return the twists (3*num_fingers) of each finger w.r.t 
        % the global inertia frame viewed from finger's own body frame.
        function [finger_twists] = GetFingerBodyTwistsWrtInertiaFrame(obj)
            finger_twists_wrt_hand_body = obj.GetFingerBodyTwistsWrtHand();
            hand_bodytwists_wrt_inertia_body = obj.GetHandBodyTwistWrtInertiaFrame();
            finger_carts = obj.GetFingerCartesiansWrtHand();
            finger_twists = zeros(3, obj.num_fingers);
            for i = 1:1:obj.num_fingers
                finger_twists(:, i) = SE2Algebra.GetBodyTwistABCChain(...
                    hand_bodytwists_wrt_inertia_body, finger_twists_wrt_hand_body(:, i), finger_carts);
            end
        end
        
        function [finger_twists, finger_carts] = GetFingerGlobalTwistsAndCartesianWrtInertiaFrame(obj)
            finger_body_twists = obj.GetFingerBodyTwistsWrtInertiaFrame();
            finger_carts = obj.GetGlobalFingerCartesians();
            finger_twists = SE2Algebra.TransformTwistFromLocalToGlobal(finger_body_twists, finger_carts);
        end
        
        function [hand_twist] = GetHandBodyTwistWrtInertiaFrame(obj)
            % For body twist: the angular part is thetadot. 
            % The linear part is R'*pdot. 
            hand_twist = SE2Algebra.GetBodyTwistGivenQandQdot(obj.q, obj.qdot);
        end
        
        function [hand_twist] = GetHandGlobalTwistWrtInertiaFrame(obj)
            hand_twist = SE2Algebra.GetGlobalTwistGivenQandQdot(obj.q, obj.qdot);
        end
        
        function [] = Draw(obj, h, q, c)
            if (nargin < 3)
                q = obj.q;
            end
            if (nargin < 4)
                c = 'k';
            end
            % Draw round fingers given (optional) config q and color.
            figure(h);
            hold on;
            q_tmp = obj.q;
            obj.q = q;
            finger_carts_global = obj.GetGlobalFingerCartesians();
            if strcmp(obj.finger_type, 'all_circles')
                for i = 1:1:obj.num_fingers
                    drawCircle(finger_carts_global(1,i), finger_carts_global(2,i), obj.finger_radius, 'color', c);
                end
            elseif strcmp(obj.finger_type, 'multi_polygons')
                % Draw each polygons.
                for i = 1:1:obj.num_fingers
                    finger_vertices = GetPolygonShapeInWorldFrame(...
                        obj.finger_geometries{i}, finger_carts_global(:,i));
                    drawPolygon(finger_vertices', c);
                end
            end
            obj.q = q_tmp;
        end
        
    end
    
end

