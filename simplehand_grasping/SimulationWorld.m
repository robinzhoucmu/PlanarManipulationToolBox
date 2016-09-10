classdef SimulationWorld
    properties
        % Class instance of PushObject (or object that needs to be grasped.
        pushobj
        % Finger trajectory generation class.
        finger_traj
    end
    
    methods (Access = public)
        function obj = SimulationWorld(pushobj, finger_traj, mu, finger_radius, num_fingers)
        % Constructor function. Assume PushObject and PitchCompute classes
        % are already properly initialized.
        % mu: coefficient of friction between object and pusher finger.
            obj.pushobj = pushobj;
            obj.finger_traj = finger_traj;
            obj.mu = mu;
            if (nargin < 4)
                finger_radius = 0.005;
            end
            if (nargin < 5)
                num_fingers = 3;
            end
            obj.finger_radius = finger_radius;
            obj.num_fingers = num_fingers;
        end
        
        function SimulationRollOut(obj)
            flag_finish = false;
            maxT = max(obj.finger_traj.t);
            cur_t = 0;
            opts = odeset('Events', @obj.FingerTouchObjectEvent);
            while ~flag_finish
                t_range = cur_t:maxT;
                sol = ode45(@obj.finger_traj.spirofun, t_range, [obj.finger_traj.INITIAL_R; 0], opts);
                % Resolve finger pushing event.
                cur_t = sol.x(end);
                dt = cur_t - sol.x(end-1);
                num_finger_touches = length(sol.ie);
                % Store the finger state right at collision.
                pt = sol.y(:,end);
                twist_linear = obj.finger_traj.compute_vels(cur_t, pt);
                pt_fingers = zeros(2,num_finger_touches);
                twist_fingers = zeros(3, num_finger_touches);
                for i = 1:1:num_finger_touches
                    rot_angle = 2 * pi * (sols.ie(i) - 1.0) / obj.num_fingers;
                    R = [cos(rot_angle), -sin(rot_angle); sin(rot_angle), cos(rot_angle)];
                    pt_fingers(:,i) = R * pt;
                    twist_fingers(:, i) = [R * twist_linear; 0];
                end
                % If only one finger is involved, simply use the pushing
                % motion model. 
                if (num_finger_touches == 1)
                    % Get the position and twist of the touching finger.
                [flag_contact, pt_contact, vel_contact, outward_normal_contact] = ...
                  obj.pushobj.GetRoundFingerContactInfo(pt_fingers(:,1), obj.finger_radius, twist_fingers(:,1));
                [twist_local, wrench_load_local, contact_mode] = ...
                    obj.pushobj.ComputeVelGivenPointRoundFingerPush(obj,pt_contact, vel_contact, outward_normal_contact, obj.mu);
                %twist_global = SE2Algebra.TransformTwistFromLocalToGlobal(twist_local, obj.pushobj.pose(3));
                %mat_exp_twist =  SE2Algebra.GetExponentialMapGivenTwistVec(twist_global * dt);
                homogT = SE2Algebra.GetExponentialMapGivenTwistVec(twist_local * dt);
                cur_homogT = SE2Algebra.GetHomogTransfFromCartesianPose(obj.pushobj.pose) * homogT; 
                obj.pushobj.pose = SE2Algebra.GetCartesianPoseFromHomogTransf(cur_homogT);
                
                elseif (num_finger_touches == 2)
                % Otherwise if 2 fingers are involved, check if the object
                % will be jammed or not. Also check if it's inside the
                % caging area or not if the hand has 3 fingers. 
                else
                end
                if (cur_t == maxT)
                    flag_finish = true;
                end
            end
        end
        
        function [value, isterminal, direction] = FingerTouchObjectEvent(obj, t, x)
            for i = 1:1:obj.num_fingers
                rot_angle = 2 * pi * (i-1.0) / obj.num_fingers;
                R = [cos(rot_angle), -sin(rot_angle); sin(rot_angle), cos(rot_angle)];
                pt_finger = R * x;
                [dist, pt_closest] = obj.pushobj.FindClosestPointAndDistanceWorldFrame(pt_finger);
                ratio_penetration = 0.05;
                value(i) = 1;
                if (abs(dist - obj.finger_radius) / obj.finger_radius) < ratio_penetration
                    value(i) = 0;
                end
                isterminal(i) = 1;
                direction(i) = -1;
            end
        end
    end
    
end

