classdef SimulationWorld < handle
    properties
        % Class instance of PushObject (or object that needs to be grasped.
        pushobj
        % Finger trajectory generation class.
        finger_traj
        num_fingers
        finger_radius
        mu
        sp
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
        
        function [pose_log] = SimulationRollOut(obj)
            flag_finish = false;
            pose_log = [];
            maxT = max(obj.finger_traj.t);
            cur_t = 0;
            opts = odeset(  'RelTol',1e-6,...
                        'AbsTol', 1e-6,...
                        'Events', @obj.FingerTouchObjectEvent,...
                        'Vectorized',true,...
                        'MaxStep',0.05);    
            dt = 0.001;
            cur_finger_pos = [obj.finger_traj.INITIAL_R; 0];
            while ~flag_finish
                cur_t
                pose_log = [pose_log, obj.pushobj.pose];
                t_range = cur_t:dt:maxT;
                sol = ode45(@obj.spirofun, t_range, cur_finger_pos, opts);
                % Resolve finger pushing event.
                cur_t = sol.x(end);
                sol.x
                sol.ie
                num_finger_touches = length(sol.ie)
                % Store the finger state right at collision.
                cur_finger_pos = sol.y(:,end)
                twist_linear = obj.finger_traj.compute_vels(cur_t, cur_finger_pos)
                pt_fingers = zeros(2,num_finger_touches);
                twist_fingers = zeros(3, num_finger_touches);
                for i = 1:1:num_finger_touches
                    rot_angle = 2 * pi * (sol.ie(i) - 1.0) / obj.num_fingers;
                    R = [cos(rot_angle), -sin(rot_angle); sin(rot_angle), cos(rot_angle)];
                    pt_fingers(:,i) = R * cur_finger_pos;
                    twist_fingers(:, i) = [R * twist_linear; 0];
                end
                % If only one finger is involved, simply use the pushing
                % motion model. 
                if (num_finger_touches == 1)
                    % Get the position and twist of the touching finger.
                    [flag_contact, pt_contact, vel_contact, outward_normal_contact] = ...
                      obj.pushobj.GetRoundFingerContactInfo(pt_fingers(:,1), obj.finger_radius, twist_fingers(:,1))
                    [twist_local, wrench_load_local, contact_mode] = ...
                        obj.pushobj.ComputeVelGivenPointRoundFingerPush(pt_contact, vel_contact, outward_normal_contact, obj.mu)
                    %twist_global = SE2Algebra.TransformTwistFromLocalToGlobal(twist_local, obj.pushobj.pose(3));
                    %mat_exp_twist =  SE2Algebra.GetExponentialMapGivenTwistVec(twist_global * dt);
                    homogT = SE2Algebra.GetExponentialMapGivenTwistVec(twist_local * 0.01)
                    cur_homogT = SE2Algebra.GetHomogTransfFromCartesianPose(obj.pushobj.pose) * homogT
                    obj.pushobj.pose = SE2Algebra.GetCartesianPoseFromHomogTransf(cur_homogT);
                    obj.pushobj.pose
                
                elseif (num_finger_touches == 2)
                % Otherwise if 2 fingers are involved, check if the object
                % will be jammed or not. Also check if it's inside the
                % caging area or not if the hand has 3 fingers. 
                else
                end
                if (cur_t >= maxT - 0.01)
                    flag_finish = true;
                end
            end
        end
        
        function [value, isterminal, direction] = FingerTouchObjectEvent(obj, t, x)
            for i = 1:1:obj.num_fingers
                rot_angle = 2 * pi * (i-1.0) / obj.num_fingers;
                R = [cos(rot_angle), -sin(rot_angle); sin(rot_angle), cos(rot_angle)];
                pt_finger = R * x
                [pt_closet, dist] = obj.pushobj.FindClosestPointAndDistanceWorldFrame(pt_finger)
                ratio_penetration = 0.05;
                value(i) = 1;
%                 r = abs(dist - obj.finger_radius) / obj.finger_radius
%                 if (r < ratio_penetration)
%                     value(i) = 0;
%                 end
                if (dist < obj.finger_radius)
                    value(i) = 0;
                end
                isterminal(i) = 1;
                direction(i) = -1;
            end
        end
        
      function dpdt = spirofun(obj, t,p)
        dpdt = obj.finger_traj.compute_vels(t,p);
      end
      
    end
    
end

