classdef PushActionDubins < handle
    properties
        % diagnoal approximation of limit surface at the push point wrench
        % cone. A = diag(a, a, b) where b is un-normalized. 
        ls_a
        ls_b
        mu
        % push point (x,y) in local frame. 
        pt
        % inward normal (n_x, n_y) in local frame. 
        np
        % directions of the left and right edges of the friction cone. 
        fl
        fr
        % the dubins push frame (rc_x, rc_y, rc_theta): origin -> center of rear axle
        % +y axis -> the vector pointing from the contact point to COM.
        rc
        % the unit length. a/b
        unit_length
        % boolean tag whether the contact point is symmetric.
        flag_symmetric;
    end
    methods (Access = public)
        % Set the pushing point (2*1), normal (2*1 unit) in local object frame and friction.
        function obj =  PushActionDubins(pt, np, mu)
            obj.pt = pt;
            obj.np = np;
            obj.mu = mu;
            % Compute the left and right edges of the friction cone. 
            cone_half_angle = atan2(mu, 1);
            % Compute the friction cone edges. 
            Rl = [cos(cone_half_angle), -sin(cone_half_angle); sin(cone_half_angle), cos(cone_half_angle)];
            Rr = [cos(-cone_half_angle), -sin(-cone_half_angle); sin(-cone_half_angle), cos(-cone_half_angle)];
            obj.fl = Rl * obj.np;
            obj.fr = Rr * obj.np;
            % If the inward normal is the opposite of point vector, then
            % it's symmetric.
            if p_x * n_y + p_y * n_x == 0
                obj.flag_symmetric = 1;
            else
                obj.flag_symmetric = 0;
            end
        end
        
        % Set the diagonal limit surface approximation parameters.
        function [] = SetLimitSurfaceParameters(obj, ls_a, ls_b)
            obj.ls_a = ls_a;
            obj.ls_b = ls_b;
            % Compute the characteristic length.
            obj.unit_length = ls_a / ls_b;
        end
        
        % Compute the center of rear axle: the mid point between the two
        % CORs corresponding to left and right edge of the friction cone.
        function [] = ComputeCenterOfRearAxle(obj)
            % Compute distances of the line of forces to the COM. 
            % Equation for left cone: fl_y * x - fl_x * y - fl_y * p_x  + fl_x * p_y     
            dl = abs(-obj.fl(2) * obj.pt(1) + obj.fl(1) * obj.pt(2)) / norm(obj.fl);
            dr = abs(-obj.fr(2) * obj.pt(1) + obj.fr(1) * obj.pt(2)) / norm(obj.fr);
            % Compute the distance of the contact point to the COM.
            dpt = norm(obj.pt);
            % Compute the two CORs.
            cor_dy = obj.unit_length / dpt; 
            cor_l_dx = sqrt((obj.unit_length / dl)^2 - cor_dy^2);
            cor_r_dx = sqrt((obj.unit_length / dr)^2 - cor_dy^2);
            cor_l = [cor_l_dx ; cor_dy];
            cor_r = [-cor_l_dy ; cor_dy];
            % Use the mid point as the "shifted" center of rear axle. 
            
        end
        
        % Given the current object local frame, return the dubin push frame.
        function [pose_pushframe] = GetDubinPushFrameGivenLocalFrame(pose_localframe)
        end

        function [pose_dubinframe] = GetLocalFrameGivenDubinPushFrame(pose_pushframe)
        end

        % Convert the flat output to cartesian push frame pose. 
        function [ cart_pushframe] = FlatSpaceToCartesianSpace(z, vz)
        
        end
        
        % This is for converting the start and end pose to flat space. Assuming instantaneous velocity will follow the positive local y axis. 
        function CartesianSpaceToFlatSpace(cart_pushframe, z)
        end
    end
end