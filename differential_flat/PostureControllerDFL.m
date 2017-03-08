% Posture stabilization to origin with dynamic feedback linearization.
% Fixed control frequency. States gets updated at discrete steps. 
classdef PostureControllerDFL < handle
    properties
        % Control law: x needs to converge faster.
        %u_x = -kp * z_x - kv * z_xdot;  
        %u_y = -kp*(z_y - a/br) - kv * z_ydot;
        % Position term gains
        kp
        % Velocity term gains
        kv
        % Control frequency Hz.
        freq
        % Last time for internal states update.
        t_last_update
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
        function [] = SetControlParameters(obj, freq, kp, kv, zeta0)
            obj.freq = freq;
            obj.kp = kp;
            obj.kv = kv;
            obj.zeta = zeta0;
        end
        
        function [vp] = GetControlOutput(obj)
        % Return pusher point cartesian output.
        % First, compute control in flat space. 
            ux = -obj.kp(1) * obj.z(1) - obj.kv(1) * obj.zdot(1);   
            uy = -obj.kp(2) * (obj.z(2) - obj.ls_a/(obj.ls_b * obj.r)) - obj.kv(2) * obj.zdot(2);
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
        end
        
        function [] = UpdateInternalStates(obj, t, q, qdot)
            % If current time - last update time >= 1.0 / control_frequency, 
            % we update the (perceived) cartesian state and derivative. 
            % Also, integrate the internal extended dynamics.
            delta_t = t - obj.t_last_update;
            if delta_t >= 1.0 / obj.freq
                obj.q = q;
                obj.qdot = qdot;
                obj.zeta = obj.zeta + delta_t * obj.zetadot;
                obj.t_last_update = t;
                % Update the flat output as well.
                c = obj.ls_a /  (obj.ls_b * obj.ls_r);
                obj.z(1) = obj.q(1) - c * sin(obj.q(3));
                obj.z(2) = obj.q(2) + c * cos(obj.q(3));
                obj.zdot(1) = obj.qdot(1) - c * cos(obj.q(3)) * obj.qdot(3);
                obj.zdot(2) = obj.qdot(2) - c * sin(obj.q(3)) * obj.qdot(3);
            end
        end
    end
end

