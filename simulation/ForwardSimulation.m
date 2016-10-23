classdef ForwardSimulation < handle
    % Forward simulation of a single object subject to an hand trajectory.
    properties
        pushobj
        hand_traj
        hand
        % Coefficient of friction between the object and the hand.
        mu
        dt_collision
    end
    
    methods (Access = public)
        function obj = ForwardSimulation(pushobj, hand_traj, mu, dt_collision)
            obj.pushobj = pushobj;
            obj.hand_traj = hand_traj;
            obj.mu = mu;
            if (nargin < 4)
                obj.dt_collision = 0.01;
            else
                obj.dt_collision = dt_collision;
            end
        end
        
        function [results] = RollOut(obj)
            opts = odeset('RelTol',1e-6,...
              'AbsTol', 1e-6,...
              'Events', @obj.ContactEvent,...
              %'Vectorized',true,...
              'MaxStep',0.05);
            
            hand_configs = [];
            obj_configs = [];
            cur_t = 0;
            cur_hand = obj.hand_traj.GetHandConfiguration(cur_t);
            flag_finish = false;
            % Simulate until jamming happens.
            while ~flag_finish
                horizon_max = 10;
                t_range = [cur_t cur_t + horizon_max];
                % Roll out the hand trajectory until contact happens.
                sol = ode45(@obj.HandMotion, t_range, cur_hand, opts);
                last_t = cur_t;
                cur_t = sol.x(end);
                cur_hand = sol.y(:,end);
                t_eval = last_t:0.05:cur_t;
                hand_configs(end+1:end+length(t_eval), :) = deval(sol, t_eval);
                obj_configs(end+1:end+length(t_eval), :) = obj.pushobj.pose;
                % Resolve contact. 
                
            end
        end
    end
    
    methods (Access = private)
        % velocity of the hand trajectory.
        function dx = HandMotion(obj, t, x)
            dx = obj.hand_traj.GetHandConfigurationDot(t);
        end
        % Contact event detection.
        function [values, isterminal, direction] = ContactEvent(obj, t, hand_config)
            %values = ones(obj.hand.num_fingers, 1);
            isterminal = ones(obj.hand.num_fingers, 1);
            direction = zeros(obj.hand.num_fingers, 1);
            obj.hand.q = hand_config;
            finger_poses = obj.hand.GetGlobalFingerPositions();
            [pt_closest, dist] = obj.pushobj.FindClosestPointAndDistanceWorldFrame(finger_poses);
            values = max(dist - obj.hand.finger_radius, 0);
        end
        
        % Resolves contact at time t.
        % 1) Compute the contact points and velocity.
        % 2) If single point contact, call pushing model.
        % 4) If multiple point contacts, detect if jammed or properly
        % grasped. Otherwise, the object can be moved, return the movement
        % of the object.
        function [contact_info, twist_obj] = ContactResolution(obj, hand_q, hand_qdot, obj_pose)
            obj.hand.SetQandQdot(hand_q, hand_qdot);
            % Get finger poses and twists in global frame. 
            [finger_twists, finger_carts] = obj.hand.GetFingerGlobalTwistsAndCartesianWrtInertiaFrame();
            % Extract fingers that are in contact with the object.
            contact_values = obj.ContactEvent([], hand_q);
            contact_info.finger_index_contact = find(contact_values == 0);
            contact_info.num_fingers_contact = length(finger_index_contact);
            if (contact_info.num_fingers_contact == 1)
                
            elseif (contact_info.num_fingers_contact > 1)
                % Multi-contact resolution.
            else
                error('ODE detects contact yet no contact has been identified.')
            end
        end
    end
    
end

