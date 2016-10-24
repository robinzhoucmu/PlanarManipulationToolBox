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
              'MaxStep',0.05);
              %'Vectorized',true,...
             
            dt_record = 0.05;
            results.hand_configs = [];
            results.obj_configs = [];
            cur_t = 0;
            cur_hand_q = obj.hand_traj.GetHandConfiguration(cur_t);
            flag_finish = false;
            t_max =  max(obj.hand_traj.t);
            % Simulate until jamming happens.
            while ~flag_finish && cur_t < t_max
                t_range = [cur_t t_max];
                % Roll out the hand trajectory until contact happens.
                sol = ode45(@obj.HandMotion, t_range, cur_hand_q, opts);
                last_t = cur_t;
                cur_t = sol.x(end);
                cur_hand_q = sol.y(:,end);
                cur_hand_qdot = obj.hand_traj.GetHandConfigurationDot(cur_t);
                
                t_eval = last_t:dt_record:cur_t;
                results.hand_configs(end+1:end+length(t_eval), :) = deval(sol, t_eval);
                % Till the contact, the object is remaining static.
                results.obj_configs(end+1:end+length(t_eval), :) = obj.pushobj.pose;
                % Resolve contact. 
                [contact_info] = obj.ContactResolution(cur_hand_q, cur_hand_qdot);
                contact_info.t = cur_t;
                results.all_contact_info{end+1} = contact_info;
                % If object cannot be moved (i.e., jammed or grasped), then
                % we terminate the rollout.
                if (~strcmp(contact_info.obj_status, 'pushed'))
                    flag_finish = true;
                end
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
            finger_poses = obj.hand.GetGlobalFingerCartesians();
            [pt_closest, dist] = obj.pushobj.FindClosestPointAndDistanceWorldFrame(finger_poses);
            values = max(dist - obj.hand.finger_radius, 0);
        end
        
        % Resolves contact at time t.
        % 1) Compute the contact points and velocity.
        % 2) If single point contact, call pushing model.
        % 4) If multiple point contacts, detect if jammed or properly
        % grasped. Otherwise, the object can be moved, return the movement
        % of the object.
        function [contact_info] = ContactResolution(obj, hand_q, hand_qdot)
            contact_info.hand_q = hand_q;
            contact_info.hand_qdot = hand_qdot;
            % Object pose at the momemnt of contact.
            contact_info.obj_pose_prev = obj.pushobj.pose;
            obj.hand.SetQandQdot(hand_q, hand_qdot);
            % Get finger poses and twists in global frame. 
            [finger_twists, finger_carts] = obj.hand.GetFingerGlobalTwistsAndCartesianWrtInertiaFrame();
            % Extract fingers that are in contact with the object.
            contact_values = obj.ContactEvent([], hand_q);
            contact_info.finger_index_contact = find(contact_values == 0);
            contact_info.num_fingers_contact = length(finger_index_contact);
            contact_info.finger_carts_contact = finger_carts(contact_info.finger_index_contact);
            contact_info.finger_twists_contact = finger_twists(contact_info.finger_index_contact);
            contact_info.pt_contact = zeros(2, contact_info.num_fingers_contact);
            contact_info.vel_contact = zeros(2, contact_info.num_fingers_contact);
            contact_info.outward_normal_contact = zeros(2, contact_info.num_fingers_contact);
            
            if (contact_info.num_fingers_contact == 1)
                % Get the position, velocity and contact normal of the touching finger.
                [flag_contact, contact_info.pt_contact, vel_contact, outward_normal_contact] = ...
                obj.pushobj.GetRoundFingerContactInfo(contact_info.finger_carts_contact, obj.hand.finger_radius, contact_info.finger_twists_contact);
                % Compute the object twist and contact mode using the
                % pushing motion model.
                [contact_info.twist_local, contact_info.wrench_local, contact_info.contact_mode] = ...
                       obj.pushobj.ComputeVelGivenPointRoundFingerPush(pt_contact, vel_contact, outward_normal_contact, obj.mu);
                contact_info.obj_status = 'pushed';
            elseif (contact_info.num_fingers_contact > 1)
                % Multi-contact resolution.
            else
                error('ODE detects contact yet no contact has been identified.')
            end
            
            % Update the object pose.
            contact_info.twist_global = SE2Algebra.TransformTwistFromLocalToGlobal(contact_info.twist_local, obj.pushobj.pose);
            contact_info.obj_center_vel = SE2Algebra.GetTwistMatrix(contact_info.twist_global) * [obj.pushobj.pose(1:2);1];
            nxt_homog_trans =  SE2Algebra.GetHomogTransfFromCartesianPose(obj.pushobj.pose) * SE2Algebra.GetExponentialMapGivenTwistVec(twist_local * obj.dt_collision);
            obj.pushobj.pose = SE2Algebra.GetCartesianPoseFromHomogTransf(nxt_homog_trans);
            contact_info.obj_pose_nxt = obj.pushobh.pose;
        end
        
    end
    
end

