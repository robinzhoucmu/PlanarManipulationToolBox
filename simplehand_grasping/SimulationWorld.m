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
        
        function [flags, pose_log] = SimulationRollOut(obj, flag_plot)
            if (nargin == 1)
                flag_plot = false;
            end
            % Default flags.
            flags.jammed = 0;
            flags.grasped = 0;
            flags.missed = 1;
            
            flag_finish = false;
            pose_log = [];
            maxT = max(obj.finger_traj.t);
            cur_t = 0;
            opts = odeset('RelTol',1e-6,...
                          'AbsTol', 1e-6,...
                          'Events', @obj.FingerTouchObjectEvent,...
                          'Vectorized',true,...
                          'MaxStep',0.05);    
            dt_collision = 0.005;
            [cur_finger_pos,~] = obj.finger_traj.get_pos_and_vel(cur_t);
            if (flag_plot)
                figure;
            end
            k = 0;
            plot_interval_t = 0.02;
            pt_all_fingers = zeros(2, obj.num_fingers);
            twist_all_fingers = zeros(3, obj.num_fingers);
            while ~flag_finish
                pose_log = [pose_log, obj.pushobj.pose];
                t_range = [cur_t maxT];
                sol = ode45(@obj.spirofun, t_range, cur_finger_pos, opts);
                % Resolve finger pushing event.
                cur_t = sol.x(end);
                % Store the finger state right at collision.
                cur_finger_pos = sol.y(:,end);
                twist_linear = obj.finger_traj.compute_vels(cur_t, cur_finger_pos);
                % I don't know why but ode is not recording simulatanoues
                % event, instead it only records the first finger.
                touch_values = obj.FingerTouchObjectEvent(cur_t, cur_finger_pos);
                sol.ie = find(touch_values == 0);
                num_finger_touches = length(sol.ie);

                % Get All fingers location and velocity.
                for i = 1:1:obj.num_fingers
                    rot_angle = 2 * pi * (i - 1.0) / obj.num_fingers;
                    R = [cos(rot_angle), -sin(rot_angle); sin(rot_angle), cos(rot_angle)];      
                    pt_all_fingers(:,i) = R * cur_finger_pos;
                    twist_all_fingers(:, i) = [R * twist_linear; 0];
                end 
                pt_fingers = pt_all_fingers(:, sol.ie);
                twist_fingers = twist_all_fingers(:, sol.ie);
                if (flag_plot)
                    % Drawing
                    if cur_t > k * plot_interval_t
                        drawCircle(obj.pushobj.pose(1), obj.pushobj.pose(2), obj.pushobj.shape_parameters.radius, 'k');
                        plot(obj.pushobj.pose(1), obj.pushobj.pose(2), 'k+');
                        hold on;
                        % Draw all fingers.
                        for i = 1:1:obj.num_fingers
                            drawCircle(pt_all_fingers(1,i), pt_all_fingers(2,i), obj.finger_radius, 'r');
                        end
                         k = k + 1;
                    end
                end
                flag_contact = zeros(num_finger_touches, 1);
                pt_contact = zeros(2, num_finger_touches);
                vel_contact = zeros(2, num_finger_touches);
                outward_normal_contact = zeros(2, num_finger_touches);
                % If only one finger is involved, simply use the pushing
                % motion model. 
                if (num_finger_touches == 1)
                    % Get the position and twist of the touching finger.
                    [flag_contact, pt_contact, vel_contact, outward_normal_contact] = ...
                      obj.pushobj.GetRoundFingerContactInfo(pt_fingers(:,1), obj.finger_radius, twist_fingers(:,1));
                    [twist_local, wrench_load_local, contact_mode] = ...
                        obj.pushobj.ComputeVelGivenPointRoundFingerPush(pt_contact, vel_contact, outward_normal_contact, obj.mu);
                    %twist_global = SE2Algebra.TransformTwistFromLocalToGlobal(twist_local, obj.pushobj.pose(3));
                    %mat_exp_twist =  SE2Algebra.GetExponentialMapGivenTwistVec(twist_global * dt);
                    homogT = SE2Algebra.GetExponentialMapGivenTwistVec(twist_local * dt_collision);
                    cur_homogT = SE2Algebra.GetHomogTransfFromCartesianPose(obj.pushobj.pose) * homogT;
                    obj.pushobj.pose = SE2Algebra.GetCartesianPoseFromHomogTransf(cur_homogT);
                
                elseif (num_finger_touches == 2)
                % Otherwise if 2 fingers are involved, check if the object
                % will be jammed or not. Also check if it's inside the
                % caging area or not if the hand has 3 fingers. 
                    for i = 1:1:num_finger_touches
                       % Get each contact point, velocity
                       [flag_contact(i), pt_contact(:,i), vel_contact(:,i), outward_normal_contact(:,i)] = ...
                          obj.pushobj.GetRoundFingerContactInfo(pt_fingers(:,i), obj.finger_radius, twist_fingers(:,1));
                    end                
                    % If neither is breaking contact, check for jamming.
                    if (flag_contact(1) && flag_contact(2))
                        flag_jammed = obj.pushobj.CheckForTwoContactsJammingGeometry(...
                            pt_contact, outward_normal_contact, [obj.mu;obj.mu]);

                        if (flag_jammed)
                            flags.jammed = 1;
                            flags.missed = 0;
                        else
                            [flag_cagged, flag_in, flag_on] = obj.pushobj.CheckForCagingGeometry(pt_all_fingers, obj.finger_radius);
                            if flag_cagged
                                flags.grasped = 1;
                                flags.missed = 0;
                            end
                        end
                        flag_finish = true;
                    end
                else
                end
                if (cur_t >= maxT - 0.01)
                    flag_finish = true;
                end
            end
            % Check for final pose. 
            if (~flags.jammed)
                [flag_cagged, flag_in, flag_on] = obj.pushobj.CheckForCagingGeometry(pt_all_fingers, obj.finger_radius);
                if flag_cagged
                    flags.grasped = 1;
                    flags.missed = 0;
                end
            end
            axis equal;
        end
        
        function [value, isterminal, direction] = FingerTouchObjectEvent(obj, t, x)
            value = ones(obj.num_fingers, 1);
            isterminal = ones(obj.num_fingers, 1);
            direction = zeros(obj.num_fingers, 1);
            for i = 1:1:obj.num_fingers
                rot_angle = 2 * pi * (i-1.0) / obj.num_fingers;
                R = [cos(rot_angle), -sin(rot_angle); sin(rot_angle), cos(rot_angle)];
                pt_finger = R * x;
                [pt_closet, dist] = obj.pushobj.FindClosestPointAndDistanceWorldFrame(pt_finger);
                ratio_penetration = 0.05;
%                 r = abs(dist - obj.finger_radius) / obj.finger_radius
%                 if (r < ratio_penetration)
%                     value(i) = 0;
%                 end
%                 if (dist <= obj.finger_radius)
%                     value(i) = 0;
%                 end
                value(i) = max(dist - obj.finger_radius,0);
            end

        end
        
      function dpdt = spirofun(obj, t,p)
        dpdt = obj.finger_traj.compute_vels(t,p);
      end
      
    end
    
end

