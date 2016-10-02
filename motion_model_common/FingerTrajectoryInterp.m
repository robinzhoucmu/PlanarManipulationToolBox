classdef FingerTrajectoryInterp
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
        function obj = SetPositionOverTime(obj, t, p)
            obj.t = t;
            obj.p = p;
        end
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
        end
        function [p, v] = GetPositionVelocity(obj, t)
            p = ppval(obj.interp_poly, t)';
            v = ppval(obj.interp_poly_der, t)';
        end
        
    end
end