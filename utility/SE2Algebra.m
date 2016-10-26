classdef SE2Algebra < handle
    
    properties
    end
    
    methods(Static) % By default, input are column based.
        
        function [mat_twist] = GetTwistMatrix(twist)
            % Given 3d vector representation of twist [vx;vy;omega] , 
            % return 3*3 matrix representation. 
            mat_twist = [0, -twist(3), twist(1);
                        twist(3), 0, twist(2);
                        0, 0, 1];
        end
        
        function [mat_exp_twist] = GetExponentialMapGivenTwistMat(mat_twist)
            % Given twist matrix, return corresponding homogTransf via
            % exponential map.
            theta = mat_twist(2,1);
            R = [cos(theta), -sin(theta);
                sin(theta), cos(theta)];
            if theta~=0
                % http://www.ethaneade.org/lie.pdf 
                V = (1.0 / theta) * [sin(theta), -(1-cos(theta)); (1-cos(theta)), sin(theta)];
                mat_exp_twist = [R, V * mat_twist(1:2,end);0,0,1];
            else
                mat_exp_twist = [eye(2,2), mat_twist(1:2,end);0,0,1];
            end
        end
        
        function [mat_exp_twist] = GetExponentialMapGivenTwistVec(twist)
            % Given twist vector, return corresponding homogTransf via
            % exponential map.
            mat_twist = SE2Algebra.GetTwistMatrix(twist);
            mat_exp_twist = SE2Algebra.GetExponentialMapGivenTwistMat(mat_twist);
        end
        
        function [twist_global] = TransformTwistFromLocalToGlobal(twist_local, cart_pose)
            % Given twist (3*1) in local body frame and cartesian pose of 
            % the body in global frame, return the twist in global frame 
            % via Adjoint Mapping.
            theta = cart_pose(3);
            R = [cos(theta), -sin(theta);
                sin(theta), cos(theta)];
            Adj = [R, [cart_pose(2); -cart_pose(1)]; 0,0,1];
            twist_global = Adj * twist_local;
        end
        
        function [mat_homogtransf] = GetHomogTransfFromCartesianPose(cart_pose)
            % Given cartesian pose, return the 3*3 homogtransf matrix
            theta = cart_pose(3);
            R = [cos(theta), -sin(theta);
                sin(theta), cos(theta)];
            mat_homogtransf = [R, cart_pose(1:2); 0, 0, 1];
        end
        
        function [cart_pose] = GetCartesianPoseFromHomogTransf(mat_homogtransf)
            % Given homogtransformation matrix, return 3d pose vector.
            theta = atan2(mat_homogtransf(2,1), mat_homogtransf(1,1));
            cart_pose = [mat_homogtransf(1:2,3); theta];
        end
        
        function [cart_ac, g_ac] = GetCartPoseABCChain(cart_ab, cart_bc)
            % Give the cartesian pose of b w.r.t a and c w.r.t b
            % Compute the cartesian pose c w.r.t a. 
            g_ac = SE2Algebra.GetHomogTransfFromCartesianPose(cart_ab) * SE2Algebra.GetHomogTransfFromCartesianPose(cart_bc);
            cart_ac = SE2Algebra.GetCartesianPoseFromHomogTransf(g_ac);
        end
        
        function [twist_ac] = GetBodyTwistABCChain(twist_ab, twist_bc, cart_pose_bc)
            % Body twist: V_{ac} = Ad_{g_bc}^{-1}V_{ab} + V_{bc}
            R = [cos(cart_pose_bc(3)), -sin(cart_pose_bc(3));
                sin(cart_pose_bc(3)), cos(cart_pose_bc(3))];
            % Adjoint inverse: [R', -R'*[y;x]; 0,0,1];
            Adj_bc_inv = [R', -R'*[cart_pose_bc(2); -cart_pose_bc(1)];
                          0,0,1];
            twist_ac = Adj_bc_inv * twist_ab + twist_bc;
        end
        
        function [twist_ac] = GetGlobalTwistABCChain(twist_ab, twist_bc, cart_pose_ab)
            % Spatial Twist: V_{ac} = Ad_{g_ab}V_{bc} + V_{ab}
            R = [cos(cart_pose_ab(3)), -sin(cart_pose_ab(3));
                sin(cart_pose_ab(3)), cos(cart_pose_ab(3))];
            Adj_ab = [R, [twist_ab(2);-twist_ab(1)];0,0,1];
            twist_ac = Adj_ab * twist_bc + twist_ab;
        end
        
        function [twist] = GetBodyTwistGivenQandQdot(q, qdot)
            % Given SE(2) configuration (x,y,theta wrt world frame) & dot (which is not really a velocity) 
            % Compute the corresponding body twist. 
            R = [cos(q(3)),-sin(q(3));
                sin(q(3)), cos(q(3))];
            twist = [R' * qdot(1:2);qdot(3)];
            % The following computation is wrong. Qdot is the exact change
            % of configuration x,y.
            %twist = [R' * (qdot(3) * [-q(2); q(1)] + qdot(1:2)); qdot(3)];
        end
        
        function [twist] = GetGlobalTwistGivenQandQdot(q, qdot)
            % Given SE(2) configuration & dot, compute spatial twist.
            % linear part: -[qdot(3)]^ * q(1:2) + qdot(1:2)
            % The linear part is speed of the origin point as if connected 
            % the rigid body. 
            twist = [qdot(3)*[q(2);-q(1)] + qdot(1:2); qdot(3)];
        end
        
        function [pt_global] = GetPointsInGlobalFrame(pt, cart)
            % Input: points in local frame (2 * N) and the cartesian pose
            % of the local frame w.r.t the global frame.
            % Output: points in global frame. 2 * N.
            R = [cos(cart(3)), -sin(cart(3));
                    sin(cart(3)), cos(cart(3))];
            pt_global = bsxfun(@plus, R * pt, cart(1:2)); 
        end
    
    end
end

