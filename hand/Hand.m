classdef Hand < handle
    % Hand class, contains hand kinematics and finger geometry.
    % We assume each finger is either a round circle or a polygon. 
    properties
        % Cell array of fingers storing the geometries. 
        fingers
        % 
        q
        qdot
        % function pointer of forward kinematic in the local frame. 
        % Input of the hand configuration, return each of the finger pose.
        fun_fk     
    end
    
    methods
        function [obj] = SetQandQdot(obj, q, qdot)
            obj.q = q;
            obj.qdot = qdot;
        end
        % To call the function below, we must have set q and qdot.
        % Return the pose of each finger frame. 
        function [finger_positions] = GetFingerPositionsLocal(obj)
            finger_positions = fun_fk
        end
        % For a hand consists of round point fingers, return the velocities
        % of each finger.
        function [finger_velocities] = GetFingerTwistsLocal(obj)
        
        end
        function [finger_positions] = GetFingerPositionsGlobal(obj)
        
        end
        function [finger_velocities] = GetFingerTwistsGlobal(obj)
        
        end
    end
    
end

