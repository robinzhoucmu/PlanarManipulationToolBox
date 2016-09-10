classdef SE2Algebra
    
    properties
    end
    
    methods(Static)
        
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
            mat_twist = GetTwistMatrix(twist);
            mat_exp_twist = GetTwistMatrix(mat_twist);
        end
        
        function [twist_global] = TransformTwistFromLocalToGlobal(twist_local, cart_pose)
            % Given twist (3*1) in local body frame and cartesian pose of 
            % the body in global frame, return the twist in global frame 
            % via Adjoint Mapping.
            theta = cart_pose(3);
            R = [cos(theta), -sin(theta);
                sin(theta), cos(theta)];
            Adj = [R, [cart_pose(2); cart_pose(1)]; 0,0,1];
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
    end
    
end

