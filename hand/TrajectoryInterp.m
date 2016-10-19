classdef TrajectoryInterp < handle
    properties
        t
        p
        v
        interp_poly
        interp_poly_der
        % Interpolation mode: 
        % (1) 'pchipd': piecewise cubic hermite interpolation with derivative. 
        % The user needs to give poth position and velocity information
        % over t. 
        % (2) 'spline': smooth spline with position only.  
        % (3) 'pchip': piecewise cubic hermite with position only.
        interp_mode
    end
    methods (Access = public)
        % p is d*N.
        function obj = SetPositionOverTime(obj, t, p)
            obj.t = t;
            obj.p = p;
        end
        
        % p and v are d*N.
        function obj = SetPositionVelocityOverTime(obj, t, p, v)
            obj.t = t;
            obj.p = p;
            obj.v = v;
        end
        
        function obj = SetInterpMode(obj, interp_mode)
            obj.interp_mode = interp_mode;
        end
        
        function obj = GenerateInterpPolynomial(obj)
            if strcmp(obj.interp_mode, 'pchipd')
                  obj.interp_poly = pchipd(obj.t, obj.p, obj.v);
            elseif strcmp(obj.interp_mode, 'spline')
                  obj.interp_poly = spline(obj.t, obj.p);
            elseif strcmp(obj.interp_mode, 'pchip')
                  obj.interp_poly = pchip(obj.t, obj.p);
            else
                error('Interpolation mode not recognizable. Try pchipd, spline or pchip');
            end
            obj.interp_poly_der = fnder(obj.interp_poly);
        end
        
        function [p] = GetPosition(obj, t)
            p = ppval(obj.interp_poly, t)';
        end
        function [v] = GetVelocity(obj,t)
             v = ppval(obj.interp_poly_der, t)';
        end
        
        function [h] = PlotTrajectory(obj)
            h = figure;
            num_plot_pts = 25;
            ts = linspace(obj.t(1), obj.t(end), num_plot_pts);
            P = zeros(size(obj.p,1), num_plot_pts);
            for i = 1:1:num_plot_pts
                [P(:,i), v] = obj.GetPositionVelocity(ts(i));
            end
            plot(P(1,:), P(2,:), 'o-');
            axis equal;
        end
    end
end

% Example usage of this class: Generate a circular motion. 
% R = 0.5;
% t = 0:0.1:1;
% omega = 2*pi;
% x = R * cos(omega * t); y = R * sin(omega * t);
% dotx = -R*omega*sin(omega*t);doty = R*omega*cos(omega*t);
% p = [x;y]; v = [dotx;doty]; 
% ft = TrajectoryInterp();
% ft.SetPositionVelocityOverTime(t, p, v);
% ft.SetInterpMode('pchipd');
% ft.GenerateInterpPolynomial();
% ft.PlotTrajectory();

%----------------------------------------------%
% A slightly more complicated trajectory. Sqeeze and rotate a bit. 
% x1 = 1:-0.05:0.5; y1 = 1:-0.05:0.5;
% t1 = 0:0.05:0.5;
% t2 = 0.55:0.05:1; x2 = 0.5*sqrt(2)*cos(pi*(t2 - 0.5) + pi/4);
% y2 = 0.5*sqrt(2)*sin(pi*(t2 - 0.5) + pi/4);
% xnew = [x1,x2]; ynew = [y1,y2];
% pnew = [xnew;ynew];
% tnew = [t1,t2];
% ft.SetPositionOverTime(tnew, pnew);
% ft.GenerateInterpPolynomial()
% ft.PlotTrajectory()
