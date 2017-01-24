classdef ForwardSimulationCombinedStateNewGeometry < handle
    % Forward simulation of a single object subject to an hand trajectory.
    properties
        pushobj
        hand_traj
        controller
        hand
        % Estimated average coefficient of friction between the object and the hand.
        mu
        % Contact status
        status_contact
        % Minimum estimated coefficient of friction between the object and the hand.
        mu_min
        % Maximum estimated coefficient of friction. 
        mu_max
        % Current operating mu in each segment.
        mu_cur
    end
    
    methods (Access = public)
        function obj = ForwardSimulationCombinedStateNewGeometry(pushobj, hand_traj, hand, mu)
            obj.pushobj = pushobj;
            obj.hand_traj = hand_traj;
            obj.hand = hand;
            obj.mu = mu;
            obj.mu_cur = mu;
            % Default
            obj.mu_min = mu;
            obj.mu_max = mu;
            obj.status_contact = 'free';
        end
        
        function [results] = RollOut(obj, num_sim_segs)
            if nargin < 2
                num_sim_segs = 1;
            end
            opts = odeset('RelTol',1e-4,...
              'AbsTol', 1e-5,...
              'MaxStep',0.01);             
            dt_record = 0.02;
            %results.all_contact_info = {};
            results.hand_configs = [];
            results.obj_configs = [];
                        t_max =  max(obj.hand_traj.t);
            for ind_seg = 1:1:num_sim_segs
                t_start = t_max * (ind_seg - 1) / num_sim_segs;
                % Simulate until jamming happens.
                t_end = t_max * ind_seg  / num_sim_segs;
                %t_range = [0 t_max];
                t_range = [t_start t_end];
                %cur_t = 0;
                cur_t = t_start;
                cur_hand_q = obj.hand_traj.GetHandConfiguration(cur_t);
                % Roll out the hand trajectory until contact happens.
                obj.pushobj.InjectLSNoise();
                obj.mu_cur  = max(0, rand() * (obj.mu_max - obj.mu_min) + obj.mu_min);
                sol = ode45(@obj.ObjectHandMotion, t_range, [obj.pushobj.pose;cur_hand_q], opts);                
                %t_eval = 0:dt_record:t_max;
                t_eval = t_start:dt_record:t_end;
                all_x = deval(sol, t_eval);
                results.hand_configs(: , end+1:end+length(t_eval)) = all_x(4:end, :);
                results.obj_configs(:, end+1:end+length(t_eval)) = all_x(1:3, :);
                results.final_contact_status = obj.status_contact;
            end
        end
    end
    
    methods (Access = private)
        % velocity of the hand trajectory and the object. 
        % x is the combined state of object and hand.
        function dx = ObjectHandMotion(obj, t, x)
            dx = zeros(size(x));
            dx(4:end) = obj.hand_traj.GetHandConfigurationDot(t);        
            obj.pushobj.pose = x(1:3);
            % Set hand config and configdot.
            obj.hand.q = x(4:end);
            obj.hand.qdot = dx(4:end);
            % Check for contact or not. 
            [min_dist] = obj.pushobj.FindClosestDistanceToHand(obj.hand);
            if min_dist < obj.hand.finger_radius
                [contact_info] = obj.ContactResolutionNewGeometry(x(4:end), dx(4:end), obj.mu_cur);
                dx(1:3) = contact_info.obj_config_dot;
                if ~strcmp(contact_info.obj_status, 'pushed')
                    dx = zeros(size(x));
                end
                obj.status_contact = contact_info.obj_status;
            end
            %x
        end        
        
        function [contact_info] = ContactResolutionNewGeometry(obj, hand_q, hand_qdot, mu_rand)
              % Uniformly sample a value of coefficient of friction between
              % mu_min and mu_max.
              if (nargin < 4)
                mu_rand = max(0, rand() * (obj.mu_max - obj.mu_min) + obj.mu_min);
              end
              % Randomly inject noise 
              %obj.pushobj.InjectLSNoise();
              obj.hand.SetQandQdot(hand_q, hand_qdot);
              [contact_info.flag_contact, contact_info.pt_contact, contact_info.vel_contact, ...
                  contact_info.outward_normal_contact] = obj.pushobj.GetHandContactInfo(obj.hand);
              %contact_info.pt_contact, contact_info.vel_contact, contact_info.outward_normal_contact
              contact_info.num_contact_pts = size(contact_info.pt_contact, 2);
              contact_info.num_fingers_contact = sum(contact_info.flag_contact);
              if (contact_info.num_contact_pts == 1)
                    [contact_info.twist_local, contact_info.wrench_local, contact_info.contact_mode] = ...
                       obj.pushobj.ComputeVelGivenPointRoundFingerPush(contact_info.pt_contact,  ...
                       contact_info.vel_contact, contact_info.outward_normal_contact, mu_rand);
                    contact_info.obj_status = 'pushed';
                    contact_info.obj_config_dot = obj.GetObjectQDotGivenBodyTwist(contact_info.twist_local);
                    
              elseif (contact_info.num_contact_pts > 1)
                  %contact_info.num_fingers_contact
                  %contact_info.num_contact_pts 
                  [contact_info.twist_local, contact_info.wrench_local, flag_jammed, flag_converged] = obj.pushobj.ComputeVelGivenMultiPointRoundFingerPush(...
                        contact_info.pt_contact, contact_info.vel_contact, contact_info.outward_normal_contact, mu_rand);            
                    if ~flag_converged
                        fprintf('The multi-contact complementarity problem did not converge!\n');
                    end
                    if ~flag_jammed
                        contact_info.obj_status = 'pushed';
                        contact_info.obj_config_dot = obj.GetObjectQDotGivenBodyTwist(contact_info.twist_local); 
                        %contact_info.obj_config_dot
                    elseif contact_info.num_fingers_contact == obj.hand.num_fingers
                        contact_info.obj_status = 'grasped';
                        contact_info.obj_config_dot = zeros(3,1);
                    else
                        %contact_info.pt_contact, contact_info.vel_contact, contact_info.outward_normal_contact, obj.mu
                        contact_info.obj_status = 'jammed';
                        contact_info.obj_config_dot = zeros(3,1);
                    end
                   %contact_info.obj_status
                   %contact_info.obj_config_dot
                   %obj.pushobj.pose
              else
                  contact_info.obj_status = 'nil';
                   contact_info.obj_config_dot = zeros(3,1);
              end
              % Change back to initial limit surface coefficient.
              %obj.pushobj.RecoverInitialLS();
             
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
            contact_info.finger_index_contact = find(contact_values <= 0);
            contact_info.num_fingers_contact = length(contact_info.finger_index_contact);
            contact_info.finger_carts_contact = finger_carts(:, contact_info.finger_index_contact);
            contact_info.finger_twists_contact = finger_twists(:, contact_info.finger_index_contact);
            %contact_info.pt_contact = zeros(2, contact_info.num_fingers_contact);
            %contact_info.vel_contact = zeros(2, contact_info.num_fingers_contact);
            %contact_info.outward_normal_contact = zeros(2, contact_info.num_fingers_contact);
            
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
                %contact_info.pt_contact, contact_info.vel_contact, contact_info.outward_normal_contact
                % Compute the twist, wrench if objects will move or knowing
                % objects will be jammed. 
                [contact_info.twist_local, contact_info.wrench_local, flag_jammed, flag_converged] = obj.pushobj.ComputeVelGivenMultiPointRoundFingerPush(...
                 contact_info.pt_contact, contact_info.vel_contact, contact_info.outward_normal_contact, obj.mu);
                %contact_info.twist_local
                if ~flag_converged
                    fprintf('The multi-contact complementarity problem did not converge!\n');
                end
                if ~flag_jammed
                    contact_info.obj_status = 'pushed';
                    contact_info.obj_config_dot = obj.GetObjectQDotGivenBodyTwist(contact_info.twist_local); 
                    %contact_info.obj_config_dot
                elseif contact_info.num_fingers_contact == obj.hand.num_fingers
                    contact_info.obj_status = 'grasped';
                    contact_info.obj_config_dot = zeros(3,1);
                else
                    contact_info.obj_status = 'jammed';
                    contact_info.obj_config_dot = zeros(3,1);
                end
                %contact_info.obj_status
                %contact_info.obj_config_dot
                %obj.pushobj.pose
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
        
    end
    
end

