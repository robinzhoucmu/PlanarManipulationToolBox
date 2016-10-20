classdef HandTraj < handle
    % Class of a hand trajectory.
    properties
        % configurations of hand over t in global frame. 2*N; 
        % We follow the convention that the first 3 elements of q is the
        % local frame (x,y,theta) w.r.t the global frame.
        q 
        qdot % (optional) config velocities over t in global frame. 2*N;
        t % time. length = N. 
        interp_mode % interpolation mode. 
        traj_interp % trajectory interpolator. 
        
    end
        
    methods
        % Specify properties through opts.
        function [obj] = HandTraj(opts)
            if ~ (isfield(opts, 'q') && isfield(opts,'t'))
                error('The configurations and time need to be specified');
            end
            obj.q = opts.q;
            obj.t = opts.t;
            if ~ (isfield(opts, 'interp_mode'))
                display('The interpolation method is not defined. Use default spline method.');
                obj.interp_mode = 'spline';
            else
                obj.interp_mode = opts.interp_mode;
            end
            if (strcmp(obj.interp_mode, 'pchipd')) && ~(isfield(opts,'qdot'))
                error('The velocities need to be specified for pchipd method');
            end
            obj.qdot = opts.qdot;
            % Create the trajectory interpolator in hand configuration
            % space. 
            obj.traj_interp = TrajectoryInterp();
            obj.traj_interp.SetInterpMode(obj.interp_mode);
            if strcmp(obj.interp_mode, 'pchipd')
                obj.traj_interp.SetPositionVelocityOverTime(obj.t, obj.q, obj.qdot);
            else
                obj.traj_interp.SetPositionOverTime(obj.t, obj.q);
            end
            
        end
        % Get the configuration of the hand at time t. 
        function [qt] = GetHandConfiguration(obj, t)
            qt = obj.traj_interp.GetPosition(t);
        end
        % Get the configuration dot of the hand at time t. 
        function [qdot] = GetHandConfigurationDot(obj, t)
            qdot = obj.traj_interp.GetVelocity(t);
        end

    end
    
end

