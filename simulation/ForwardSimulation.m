classdef ForwardSimulation
    % Forward simulation of a single object subject to an hand trajectory.
    properties
        pushobj
        hand_traj
        hand
        % Coefficient of friction between the object and the hand.
        mu
    end
    
    methods (Access = public)
        function obj = ForwardSimulation(pushobj, hand_traj, mu)
        
        end
        
        function [results] = RollOut(obj)
        
        end
    end
    
    methods (Access = private)
        % velocity of the hand trajectory.
        function dx = Hand_Motion(obj, t, x)
            dx = obj.hand_traj.GetHandConfigurationDot(t);
        end
        % Contact event detection.
        function [value, isterminal, direction] = Contact_Event(obj, t, x)
            
        end
        % Resolves contact at time t.
        function [contact_info] = Contact_Resolve(obj, t)
        end
    end
    
end
