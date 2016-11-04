classdef ForwardSimulationCombinedState < handle
    % Forward simulation of a single object subject to an hand trajectory.
    properties
        pushobj
        hand_traj
        controller
        hand
        % Coefficient of friction between the object and the hand.
        mu
    end
    
    methods (Access = public)
        function obj = ForwardSimulationCombinedState(pushobj, hand_traj, hand, mu)
            obj.pushobj = pushobj;
            obj.hand_traj = hand_traj;
            obj.hand = hand;
            obj.mu = mu;

        end
        
        function [results] = RollOut(obj)
            opts = odeset('RelTol',1e-6,...
              'AbsTol', 1e-6,...
              'MaxStep',0.1);             
            dt_record = 0.02;
            results.all_contact_info = {};
            results.hand_configs = [];
            results.obj_configs = [];
            cur_t = 0;
            cur_hand_q = obj.hand_traj.GetHandConfiguration(cur_t);
            t_max =  max(obj.hand_traj.t);
            % Simulate until jamming happens.
            t_range = [0 t_max];
            % Roll out the hand trajectory until contact happens.
            sol = ode45(@obj.ObjectHandMotion, t_range, [obj.pushobj.pose;cur_hand_q], opts);                
            t_eval = 0:dt_record:t_max;
            all_x = deval(sol, t_eval);
            results.hand_configs(: , end+1:end+length(t_eval)) = all_x(4:end, :);
            % Till the contact, the object is remaining static.
            results.obj_configs(:, end+1:end+length(t_eval)) = all_x(1:3, :);
            
        end
    end
    
    methods (Access = private)
        % velocity of the hand trajectory and the object. 
        % x is the combined state of object and hand.
        function dx = ObjectHandMotion(obj, t, x)
            dx = zeros(size(x));
            dx(4:end) = obj.hand_traj.GetHandConfigurationDot(t);        
            obj.pushobj.pose = x(1:3);
            obj.hand.q = x(4:end);
            % Check for contact or not. 
            finger_poses = obj.hand.GetGlobalFingerCartesians();
            [ ~, dist] = obj.pushobj.FindClosestPointAndDistanceWorldFrame(finger_poses);
            dist = bsxfun(@minus, dist, obj.hand.finger_radius);
            if min(dist) < 0
                [contact_info] = ContactResolution(obj, x(4:end), dx(4:end), dist);
                dx(1:3) = contact_info.obj_config_dot;
            end
        end        
        % Resolves contact at time t.
        % 1) Compute the contact points and velocity.
        % 2) If single point contact, call pushing model.
        % 3) If multiple point contacts, detect if jammed or properly
        % grasped. Otherwise, the object can be moved, return the movement
        % of the object.
        function [contact_info] = ContactResolution(obj, hand_q, hand_qdot, contact_values)
            contact_info.hand_q = hand_q;
            contact_info.hand_qdot = hand_qdot;
            % Object pose at the momemnt of contact.
            contact_info.obj_pose_prev = obj.pushobj.pose;
            obj.hand.SetQandQdot(hand_q, hand_qdot);
            % Get finger poses and twists in global frame. 
            [finger_twists, finger_carts] = obj.hand.GetFingerGlobalTwistsAndCartesianWrtInertiaFrame();
            % Extract fingers that are in contact with the object.
            contact_info.finger_index_contact = find(contact_values <= 1e-6);
            contact_info.num_fingers_contact = length(contact_info.finger_index_contact);
            contact_info.finger_carts_contact = finger_carts(:, contact_info.finger_index_contact);
            contact_info.finger_twists_contact = finger_twists(:, contact_info.finger_index_contact);
            contact_info.pt_contact = zeros(2, contact_info.num_fingers_contact);
            contact_info.vel_contact = zeros(2, contact_info.num_fingers_contact);
            contact_info.outward_normal_contact = zeros(2, contact_info.num_fingers_contact);
            
            if (contact_info.num_fingers_contact == 1)
                % Get the position, velocity and contact normal of the touching finger.
                [~, contact_info.pt_contact, contact_info.vel_contact, contact_info.outward_normal_contact] = ...
                obj.pushobj.GetRoundFingerContactInfo(contact_info.finger_carts_contact(1:2), obj.hand.finger_radius, contact_info.finger_twists_contact);
                % Compute the object twist and contact mode using the pushing motion model.
                [contact_info.twist_local, contact_info.wrench_local, contact_info.contact_mode] = ...
                       obj.pushobj.ComputeVelGivenPointRoundFingerPush(contact_info.pt_contact,  ...
                       contact_info.vel_contact, contact_info.outward_normal_contact, obj.mu);
                contact_info.obj_status = 'pushed';
                contact_info.obj_config_dot = obj.GetObjectQDotGivenBodyTwist(contact_info.twist_local); 

            elseif (contact_info.num_fingers_contact > 1)
                % Multi-contact resolution.
                % Get the position, velocity and contact normal of the touching fingers. 
                [~, contact_info.pt_contact, contact_info.vel_contact, contact_info.outward_normal_contact] = ...
                obj.pushobj.GetRoundFingerContactInfo(contact_info.finger_carts_contact(1:2, :), obj.hand.finger_radius, contact_info.finger_twists_contact);
                % Compute the twist, wrench if objects will move or knowing
                % objects will be jammed. 
                [contact_info.twist_local, contact_info.wrench_local, flag_jammed, flag_converged] = obj.pushobj.ComputeVelGivenMultiPointRoundFingerPush(...
                 contact_info.pt_contact, contact_info.vel_contact, contact_info.outward_normal_contact, obj.mu);
                if ~flag_converged
                    fprintf('The multi-contact complementarity problem did not converge!\n');
                end
                if ~flag_jammed
                    contact_info.obj_status = 'pushed';
                    contact_info.obj_config_dot = obj.GetObjectQDotGivenBodyTwist(contact_info.twist_local); 
                elseif contact_info.num_finger_contact == obj.hand.num_fingers
                    contact_info.obj_status = 'grasped';
                    contact_info.obj_config_dot = zeros(3,1);
                else
                    contact_info.obj_status = 'jammed';
                    contact_info.obj_config_dot = zeros(3,1);
                end
            else
                %error('ODE detects contact yet no contact has been identified.')
            end
        end
             
        function [qdot] = GetObjectQDotGivenBodyTwist(obj, twist_body) 
            twist_global = SE2Algebra.TransformTwistFromLocalToGlobal(twist_body, obj.pushobj.pose);
            obj_center_vel = SE2Algebra.GetTwistMatrix(twist_global) * [obj.pushobj.pose(1:2);1];
            qdot = [obj_center_vel(1:2); twist_body(3)];
        end
        
%         % Update object pose given object body twist, dt equals dt_collision.
%         function [obj] = UpdateObjectPoseGivenBodyTwist(obj, twist_body)
%               nxt_homog_trans =  SE2Algebra.GetHomogTransfFromCartesianPose(obj.pushobj.pose) * ...
%               SE2Algebra.GetExponentialMapGivenTwistVec(twist_body * obj.dt_collision);
%               obj.pushobj.pose = SE2Algebra.GetCartesianPoseFromHomogTransf(nxt_homog_trans);
%         end
        
         % Contact event detection.
        function [values, isterminal, direction] = ContactEvent(obj, t, hand_config)
            isterminal = ones(obj.hand.num_fingers, 1);
            direction = zeros(obj.hand.num_fingers, 1);
            obj.hand.q = hand_config;
            finger_poses = obj.hand.GetGlobalFingerCartesians();
            [~, dist] = obj.pushobj.FindClosestPointAndDistanceWorldFrame(finger_poses);
            values = max(dist - obj.hand.finger_radius, 0);
        end
        
    end
    
end

