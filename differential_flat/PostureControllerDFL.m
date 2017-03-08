classdef PostureControllerDFL( input_args )
    properties
        % Control law: 
        %u_x = -kp * z_x - kv * z_xdot;  
        %u_y = -kp*(z_y - a/br) - kv * z_ydot;
        % Position term gains
        kpx
        kpy
        % Velocity term gains
        kvx
        kvy
        % cartesian state q:[x,y,theta]
        q
        % cartesian state derivative [x,y,thetadot]
        qdot
        % flat space z: [z_x, z_y]
        % Parameters about diagnal limit surface.
        ls_a
        ls_b
        % Parameters about pusher point
    end
    methods
        function [v_x, v_y] = ControlOutput()
        % Return pusher point cartesian output.
        end
    end

end

