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
        % center of "shifted" rear axle with respect to the rotated frame
        % at COM using the vector from push point to COM as +y axis.
        rc  
        % turning radius
        turning_radius
        % homogenous transform of push frame w.r.t local frame.
        tf_push_wrt_local
        % the unit length. a/b
        unit_length
        % boolean tag whether the contact point is symmetric.
        flag_symmetric
    end
    methods (Access = public)
        % Set the pushing point (2*1), normal (2*1 unit) in local object frame and friction.
        function obj =  PushActionDubins(pt, np, mu, ls_a, ls_b)
            obj.pt = pt;
            obj.np = np;
            obj.mu = mu;
            obj.rc = zeros(2, 1);
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
            obj.SetLimitSurfaceParameters(ls_a, ls_b);
            obj.ComputeCenterOfRearAxleAndTurningRadius();
            obj.ComputeHomogTfPushWrtLocal();
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
        function [] = ComputeCenterOfRearAxleAndTurningRadius(obj)
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
            cor_r = [-cor_r_dx ; cor_dy];
            % Use the mid point as the "shifted" center of rear axle. 
            obj.rc(1:2) = 0.5 * (cor_l + cor_r);
            obj.turning_radius = 0.5 * (cor_l_dx + cor_r_dx);
        end
        
        function [] = ComputeHomogTfPushWrtLocal(obj)
            % the dubins push frame (rc_x, rc_y, theta): origin -> center of rear axle
            % +y axis -> the vector pointing from the contact point to COM.        
            R_pushframe = [obj.np(2) , obj.np(1); - obj.np(1), obj.np(2)];
            t_pushframe = R_pushframe * obj.rc;
            obj.tf_push_wrt_local = [R_pushframe, t_pushframe; 0,0,1];
        end
        
        % Given the current object local frame, return the dubin push frame.
        function [pose_pushframe] = GetDubinPushFrameGivenLocalFrame(obj, pose_localframe)
             tf_localframe = obj.FormTfMatrixGivenPose(pose_localframe);
             tf_pushframe = tf_localframe * obj.tf_push_wrt_local;
             pose_pushframe = obj.GetPoseFromTfMatrix(tf_pushframe);
        end
        
        % Given the dubins push frame, return the object local frame.
        function [pose_localframe] = GetLocalFrameGivenDubinPushFrame(obj, pose_pushframe)
            tf_pushframe = obj.FormTfMatrixGivenPose(pose_pushframe);
            tf_localframe =  tf_pushframe * inv(obj.tf_push_wrt_local);
            pose_localframe = obj.GetPoseFromTfMatrix(tf_localframe);
        end
        
        % Helper function get tf from se2 vec. 
        function [tf] = FormTfMatrixGivenPose(obj, pose)
            tf = [cos(pose(3)), -sin(pose(3)), pose(1);
                     sin(pose(3)), cos(pose(3)), pose(2);
                     0, 0, 1];
        end
        
        % Helper function get se2 vec from tf
        function [pose] = GetPoseFromTfMatrix(obj, tf)
            pose = [tf(1:2, 3); atan2(tf(2,1), tf(1,1))];
        end
        
        % Convert the flat output z (2*N) and velocity vz (2*N) to cartesian push frame pose. 
        function [cart_pushframe] = FlatSpaceToCartesianSpace(obj, z, vz)
            cart_pushframe = [z(1,:); z(2,:); atan2(-vz(1,:), vz(2,:))];
        end
        
        % This is for converting the start and end pose to flat space. Assuming instantaneous velocity will follow the positive local y axis. 
        % the third dimension of z is heading. 
        function [z] = CartesianSpaceToFlatSpace(obj, cart_pushframe)
            z = [cart_pushframe(1,:); cart_pushframe(2,:); bsxfun(@plus, cart_pushframe(3,:), pi/2)];
        end
        
        % Given start and end pose of the object local frame w.r.t the
        % world, return the object local frame trajectories and pusher
        % point's frame (whose +y aligns with inward normal) trajectory. 
        function [traj_localframe, traj_pusherframe] = PlanDubinsPaath(obj, pose_start, pose_end, num_steps)
            % First, convert the start and end poses to DubinPushFrame. 
            pose_start_dubinsframe = obj.GetDubinPushFrameGivenLocalFrame(pose_start);
            pose_end_dubinsframe = obj.GetDubinPushFrameGivenLocalFrame(pose_end);
            % Get the flat output start and goal. 
            z_start = obj.CartesianSpaceToFlatSpace(pose_start_dubinsframe);
            z_end = obj.CartesianSpaceToFlatSpace(pose_end_dubinsframe);
            % Call Dubins curve planner. 
            % Note that z_end is not included.
            traj_z = dubins(z_start', z_end', obj.turning_radius, num_steps);
            % Add z_end.
            traj_z(:,end+1) = z_end;
            dt = 1.0 / num_steps;
            % vz = (z(i+1) - z(i)) / dt.
            v_z =  diff(traj_z, 1, 2) / dt;
            traj_dubinspushframe = obj.FlatSpaceToCartesianSpace(traj_z(1:2,1:end-1), v_z);
            % Get object local frames trajectory.
            traj_localframe = zeros(3, num_steps+1);
            traj_pusherframe = zeros(3, num_steps+1);
            for i = 1:1:size(traj_dubinspushframe, 2)
                traj_localframe(:, i) = obj.GetLocalFrameGivenDubinPushFrame(traj_dubinspushframe(:, i));
                  
            end
            % append the final goal pose. 
            traj_localframe(:, end) = pose_end;
            for i = 1:1:num_steps + 1
                R = [cos( traj_localframe(3, i)), -sin( traj_localframe(1:2, i));
                        sin( traj_localframe(1:2, i)), cos( traj_localframe(1:2, i))];
                vec_pt =   R * obj.np;
                traj_pusherframe(1:2,i) = traj_localframe(1:2, i) + vec_pt;
                traj_pusherframe(3, i) = atan2(vec_pt(1), -vec_pt(2));
            end
        end
        
    end
end