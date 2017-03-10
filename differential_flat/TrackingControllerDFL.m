% Track a trajectory with dynamic feedback linearization.
% Controller track in the flat space.
classdef TrackingControllerDFL < handle
    properties
        % Control law:
        %u_x =\ddot{z*_x} + kp * (z*_x - z_x) + kv * (\ddot{z*_x} - ddot{z_x});  
        % Trajectory (in flat space) to track.
        traj_interp
        % Position term gains
        kp
        % Velocity term gains
        kv
        % Control frequency.
        freq
        % Last update internal state time.
        t_last_update
        % Tolerance to stablize around the point of origin.
        tols
        flag_stop 
        % The integrator state.
        zeta
        zetadot
        % cartesian state q:[x,y,theta]
        q
        % cartesian state derivative [x,y,thetadot]
        qdot
        % flat space z: [z_x, z_y]
        z
        % flat space derivative: zdot: [zdot_x, z_dot_y]
        zdot
        % Parameters about diagnal limit surface. diag([ls_a, ls_a, ls_b])
        ls_a
        ls_b
        % Parameters about pusher point
        pt_x
        pt_y
        r
        % Coefficient of friction.
        mu
    end
    methods
        function [obj] = TrackingControllerDFL(freq, kp, kv, zeta0)
            obj.freq = freq;
            obj.kp = kp;
            obj.kv = kv;
            obj.zeta = zeta0;
            obj.zetadot = 0;
            obj.t_last_update = -100;
            obj.tols = [0.003; 0.003; 0.05];
            %obj.tols = [0.00; 0.00; 0.0];
            obj.flag_stop = 0;
        end
        
        function [] = SetSystemParameters(obj, ls_a, ls_b, r, mu)
            obj.ls_a = ls_a;
            obj.ls_b = ls_b;
            obj.r = r;
            obj.mu = mu;
        end
        
        function [] = SetTrackingTrajectory(obj, traj_interp)
            obj.traj_interp = traj_interp;
        end
        
        function [vp] = GetControlOutput(obj, t)
        % Return pusher point cartesian output.
            %if sum(abs(obj.q) < obj.tols) == 3
            if obj.flag_stop
                vp = [0;0];
            else
                z_ref = obj.traj_interp.GetPosition(t);
                z_ref_dot = obj.traj_interp.GetVelocity(t);
                z_ref_ddot = obj.traj_interp.GetAcceleration(t);
            % First, compute control in flat space. 
                ux = z_ref_ddot(1)  - obj.kp(1) * (obj.z(1) - z_ref(1)) - obj.kv(1) * (obj.zdot(1) - z_ref_dot(1));   
                uy = z_ref_ddot(2)  - obj.kp(2) * (obj.z(2) - z_ref(2)) - obj.kv(2) * (obj.zdot(2) - z_ref_dot(2));
            % Compute control [fx,fy] in cartesian space.
                fy = obj.zeta;  % Might need to check for fy >=0?
                fx = (-ux * cos(obj.q(3))  - uy * sin(obj.q(3))) / (obj.ls_a * obj.zeta * obj.ls_b * obj.r);  
                obj.zetadot = (-ux * sin(obj.q(3)) + uy * cos(obj.q(3))) / obj.ls_a;
            % Trim the control if fx is out of the friction cone.
                k_s = abs(fx / (fy+eps));
                ratio = obj.mu / k_s;
                if ratio < 1
                    ux = ux * ratio;
                    uy = uy * ratio;
                    fx = fx * ratio;
                    obj.zetadot = obj.zetadot * ratio;
                end
            % Convert force to velocity commands.
            % Twist V = [a*fx, a*fy, b*r*fx;]. vp = JV = [(a+ b*r^2)*fx, a*fy] 
                vp = [(obj.ls_a + obj.ls_b * obj.r^2) * fx; obj.ls_a * fy];
                % Convert to global frame.
                vp = [cos(obj.q(3)), -sin(obj.q(3)); sin(obj.q(3)), cos(obj.q(3)) ] * vp;
            end
        end
        
        function [] = UpdateInternalStates(obj, t, q, qdot)
            % If current time - last update time >= 1.0 / control_frequency, 
            % we update the (perceived) cartesian state and derivative. 
            % Also, integrate the internal extended dynamics.
            delta_t = t - obj.t_last_update;
            %if (delta_t >= 1.0 / obj.freq) && (sum(abs(q) < obj.tols) < 3)
            if (delta_t >= 1.0 / obj.freq) && ~obj.flag_stop
                obj.q = q;
                obj.q(3) = mod(obj.q(3)+2*pi, 2*pi);
                if (obj.q(3) > pi)
                    obj.q(3) = obj.q(3) - 2*pi;
                end
                obj.qdot = qdot;
                obj.zeta = obj.zeta + delta_t * obj.zetadot;
                % Update the flat output as well.
                c = obj.ls_a /  (obj.ls_b * obj.r);
                obj.z(1) = obj.q(1) - c * sin(obj.q(3));
                obj.z(2) = obj.q(2) + c * cos(obj.q(3));
                obj.zdot(1) = obj.qdot(1) - c * cos(obj.q(3)) * obj.qdot(3);
                obj.zdot(2) = obj.qdot(2) - c * sin(obj.q(3)) * obj.qdot(3);
                %display(obj.zdot)
                %display(obj.z)
                %display(obj.zeta)
                obj.t_last_update = t;
            end
             if sum(abs(obj.q) < obj.tols) == 3
                   obj.flag_stop = 1;
             end
        end
    end
end

