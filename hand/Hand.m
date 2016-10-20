classdef Hand < handle
    % Hand class, contains hand kinematics and finger geometry.
    % We assume each finger is either a round circle or a polygon. 
    properties
        % Cell array of fingers storing the geometries. 
        fingers
        num_fingers
        % 
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
        % Return the pose of each finger frame. 
        function [finger_positions] = GetFingerPositionsLocal(obj)
            finger_positions = obj.fun_fk(obj.q);
        end
        % For a hand consists of round point fingers, return the velocities
        % of each finger.
        function [finger_twists] = GetFingerTwistsLocal(obj)
            finger_twists = obj.fun_fv(obj.q, obj.qdot);
        end
        % Output: 3 * num_fingers
        function [finger_positions] = GetFingerPositionsGlobal(obj)
            finger_positions_local = obj.GetFingerPositionsLocal();
            finger_positions = zeros(3, obj.num_fingers);
            for i = 1:1:obj.num_fingers
                finger_positions(:, i) = 
            end
        end
        function [finger_velocities] = GetFingerTwistsGlobal(obj)
        
        end
    end
    
end

