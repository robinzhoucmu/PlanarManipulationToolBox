classdef PushedObject < handle
    % Class of object being pushed. 
   properties
      % Limit surface related.
      ls_coeffs
      ls_type
      % Pressure related.
      support_pts  %2*N
      pressure_weights  % N*1
      % Coefficient of contact friction between pusher and the object.
      %mu_contact
      % Shape and geometry related. 
      shape_id % unique id 
      shape_type % type for different methods of computing distance.
      shape_vertices % object coordinate frame. 2*N.
      shape_parameters % radius of circle, two axis length of ellipse, etc.
      pho % radius of gyration.
      
      % Pose related. object coordinate frame w.r.t the world frame.
      pose %3*1: [x;y;theta]
      %cur_shape_vertices % shape vertices in world frame.
      
      % Configuration parameters for Sampling of wrench,twist pairs. 
      % When the user only gives pressure points and weights for constructor 
      % function, we will sample wrench and twist pairs instead. 
      % Total number of center of rotations (CORs). 
      num_cors
      % The ratio [0,1] of CORs that will be sampled on the limit surface facet. 
      r_facet 
      
   end
   methods
       function obj = PushedObject(support_pts, pressure_weights, shape_info, ls_type, ls_coeffs) 
            obj.support_pts = support_pts;
            obj.pressure_weights = pressure_weights;

            obj.shape_id = shape_info.shape_id;
            obj.shape_type = shape_info.shape_type;
            obj.pho = shape_info.pho;
            if strcmp(obj.shape_type,'polygon')
                obj.shape_vertices = shape_info.shape_vertices;
            else
                obj.shape_parameters = shape_info.shape_parameters;
            end
            if (nargin == 5)
                obj.ls_type = ls_type;
                obj.ls_coeffs = ls_coeffs;
            else
                % If no limit surface information is provided. By default, we will fit a
                % ellipsoid model for it.
%                 obj.SetWrenchTwistSamplingConfig(200, 0.4);
%                 if (nargin == 4)
%                   obj.FitLSFromPressurePoints(ls_type);
%                 else
%                   obj.FitLSFromPressurePoints('quadratic');
%                 end
            end

       end
       function [obj] = FitLS(obj, ls_type, num_cors, r_facet)
            if (nargin < 2)
                ls_type = 'quadratic';
            end
            if (nargin < 3)
                num_cors = 200;
            end
            if (nargin < 4)
                r_facet = 0.4;  
            end
            obj.SetWrenchTwistSamplingConfig(num_cors, r_facet);
            obj.FitLSFromPressurePoints(ls_type);
       end
       function [obj] = SetWrenchTwistSamplingConfig(obj, num_cors, r_facet)
           if (num_cors < 15)
                disp('More wrench-twist points would be better.')
           end
           obj.num_cors = num_cors;
           obj.r_facet = r_facet;
       end
       
       function [obj] = FitLSFromPressurePoints(obj, ls_type)
            if (nargin == 1)
                obj.ls_type = 'quadratic';
            else
                if (strcmp(ls_type, 'quadratic') || strcmp(ls_type, 'poly4'))
                    obj.ls_type = ls_type;
                else
                    disp('limit surface type not recognized %s\n', ls_type);
                end
            end
            % Generate random points. 
            num_pts = size(obj.support_pts, 2);
            num_facet_pts = ceil(obj.r_facet * (obj.num_cors / 2) / num_pts);
            num_other_pts = ceil((1 - obj.r_facet) * (obj.num_cors / 2));
            CORs = GenerateRandomCORs3(obj.support_pts, num_other_pts, num_facet_pts);
            [F, V] = GenFVPairsFromPD(obj.support_pts, obj.pressure_weights, CORs);
            % Normalize the 3rd component of wrench and twist pairs by
            % characteristic length. Here F,V are all N*3, each row is a
            % data point. Twists are additionally normalized to unit
            % vector.
            [V, F] = NormalizeForceAndVelocities(V, F, obj.pho);
            if strcmp(obj.ls_type, 'quadratic')
                [obj.ls_coeffs, xi, delta, pred_V_dir, s] = FitEllipsoidForceVelocityCVX(F', V', 1, 5);
            elseif strcmp(obj.ls_type, 'poly4')
                [obj.ls_coeffs, xi, delta, pred_V_dir, s] = Fit4thOrderPolyCVX(F', V', 1, 5);
            end
            ls_type = obj.ls_type;
            ls_coeffs = obj.ls_coeffs;
       end
           
       function [pt_closest, dist] = FindClosestPointAndDistanceWorldFrame(obj, pt)
         % Input: pt is a 2*K column vector. 
         % Output: distance and the projected/closest point (2*K) on the object boundary.
         num_pts = size(pt, 2);
         dist = zeros(num_pts, 1); pt_closest = zeros(2, num_pts);
         theta = obj.pose(3);
         R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
         if strcmp(obj.shape_type, 'polygon')
             cur_shape = bsxfun(@plus, R * obj.shape_vertices, obj.pose(1:2));
             [tip_proj, dist] = projPointOnPolygon(pt', cur_shape');
             pt_closest = polygonPoint(cur_shape', tip_proj);
             pt_closest = pt_closest';
             
         elseif strcmp(obj.shape_type, 'circle')
             %dist = norm(pt - obj.pose(1:2));
             vec = bsxfun(@minus, pt, obj.pose(1:2));
             dist = sqrt(sum(vec.^2));
             pt_closest = obj.pose(1:2) + obj.shape_parameters.radius * bsxfun(@rdivide, vec, dist);
             dist = bsxfun(@minus, dist, obj.shape_parameters.radius);
             
         elseif strcmp(obj.shape_type, 'ellipse')
         else
            fprintf('Shape meta type not supported%s\n', obj.shape_type);
         end
      end
      
      function [vec_local] = GetVectorInLocalFrame(obj, vec) 
           % Input: a column vector 2*1 in world frame.
           % Output: rotated to local frame.
           theta = obj.pose(3);
           R = [cos(theta) -sin(theta); sin(theta) cos(theta)];
           vec_local = R' * vec;
      end
            
      function [flag_contact, pt_contact, vel_contact, outward_normal_contact] = ...
          GetRoundFingerContactInfo(obj, pt_finger_center, finger_radius, twist)
          % Input: pt_center (2*K): center of round fingers in world frame. 
          % finger_radius: radius of the round finger in meter.
          % global twist (3*K) ([vx,vy,omega]) of the finger body w.r.t world frame.
          % Output: flag_contact: whether any point of the round finger
          % will be in contact with the object.
          % pt_contact: the point (in world frame) that contacts the object.  
          % vel_contact: contact point linear velocity in world frame.
          % outward_normal: contact normal in world frame.
          num_fingers = size(pt_finger_center, 2);
          flag_contact = zeros(num_fingers, 1);
          pt_contact = zeros(2, num_fingers);
          vel_contact = zeros(2, num_fingers);
          outward_normal_contact = zeros(2, num_fingers);
 
          [pt_closest, dist] = obj.FindClosestPointAndDistanceWorldFrame(pt_finger_center); 
          r_blem = 1.00 + 1e-3;
          indices_contact = dist < (finger_radius * r_blem);
          flag_contact(indices_contact) = 1;
          pt_contact(:, indices_contact) = pt_closest(:, indices_contact);
          vel_contact(:, indices_contact) = twist(1:2, indices_contact) + bsxfun(@times, twist(3,indices_contact), [-pt_contact(2,indices_contact); pt_contact(1,indices_contact)]);
          outward_normal_contact(:, indices_contact) = pt_finger_center(:, indices_contact) - pt_contact(:, indices_contact);
          outward_normal_contact(:, indices_contact) = bsxfun(@rdivide, outward_normal_contact(:, indices_contact), sqrt(sum(outward_normal_contact(:, indices_contact).^2)) + eps);
% -----------------------------------------------          
%           if (dist <= finger_radius * r_blem)
%             flag_contact = 1;
%             pt_contact = pt_closest;
%             vel_contact = twist(1:2) + twist(3) * [-pt_contact(2);pt_contact(1)];
%             outward_normal_contact = pt_finger_center - pt_contact; 
%             outward_normal_contact = outward_normal_contact / ( eps + norm(outward_normal_contact));
%           else
%             flag_contact = 0;
%           end
      end
          
      function [twist_local, wrench_load_local, contact_mode] = ComputeVelGivenPointRoundFingerPush(obj, ...
              pt_global, vel_global, outward_normal_global, mu)
        % Input: 
        % contact point on the object (pt 2*1), pushing velocity (vel 2*1)
        % and outward normal (2*1, pointing from object to pusher) in world frame;  
        % mu: coefficient of friction. 
        % Note: Ensure that the point is actually in contact before using
        % this function. It does not check if point is on the object boundary.
        % Output: 
        % Body twist, friction wrench load (local frame) on the 1 level set of
        % limit surface and contact mode ('separation', 'sticking', 'leftsliding', 'rightsliding' ). 
        % Note that the third component is unnormalized,
        % i.e, F(3) is torque in Newton*Meter and V(3) is radian/second. 
        % If the velocity of pushing is breaking contact, then we return 
        % all zero 3*1 vectors. 
        
        % Change vel, pt and normal to local frame first. 
        vel_local = obj.GetVectorInLocalFrame(vel_global);        
        % Compute the point of contact.
        pt_local = obj.GetVectorInLocalFrame(pt_global - obj.pose(1:2));
        normal_local = obj.GetVectorInLocalFrame(outward_normal_global);
        
        [wrench_load_local, twist_local, contact_mode] = ComputeVelGivenSingleContactPtPush(vel_local, pt_local, ...
            normal_local, mu, obj.pho, obj.ls_coeffs, obj.ls_type);
        % Un-normalize the third components of F and V.
        wrench_load_local(3) = wrench_load_local(3) * obj.pho;
        twist_local(3) = twist_local(3) / obj.pho;  
        
      end
      
      function [twist, wrench, contact_modes] = ComputeVelGivenMultiPointRoundFingerPush(obj, ...
              pts_global, vels_global, outward_normals_global, mu)
        % The multi-contact 
      end
      
      function [flag_jammed] = CheckForTwoContactsJammingGeometry(obj, pts, out_normals, mus, flag_plot)
        % This function checks for jamming given two contact points.
        % Here we assume the contacts are position-controlled and are able
        % to apply infinite force to the object. 
        % It checks if the line between the two contacts lines in the 2
        % friction cones. 
        % Inputs: pt: contact points column vectors. 2*2. 
        % out_normals: outward normals at each contact point. 2*2. 
        % mus: the coefficient of frictions at the two contact points.
        % Output: boolean variable returning whether the object will be
        % jammed or not.
        if nargin < 5
            flag_plot = false;
        end
        vec_pt_12 = pts(:,2) - pts(:,1);
        pho_dummy = 1;
        fc_edges = zeros(3,4);
        fc_edges(:,1:2) = ComputeFrictionConeEdges(pts(:,1), out_normals(:,1), mus(1), pho_dummy);
        fc_edges(:,3:4) = ComputeFrictionConeEdges(pts(:,2), out_normals(:,2), mus(2), pho_dummy);
        k = zeros(4,1);
        flag_jammed = true;
        for i = 1:1:2
            vec_pt = (-1)^(i-1) * vec_pt_12;
            k(2*i-1) = vec_pt(1) * fc_edges(2, 2*i-1) - vec_pt(2) * fc_edges(1, 2*i-1);
            k(2*i) = vec_pt(1) * fc_edges(2, 2*i) - vec_pt(2) * fc_edges(1, 2*i);
            if ~(k(2*i-1) >= 0 && k(2*i) <=0)
                flag_jammed = false;
            end
            %arrow_length = obj.shape_parameters.radius * 0.5;
            arrow_length = obj.pho * 0.5;
            if (flag_plot)
                hold on;
                plot([pts(1,i), pts(1,i) + fc_edges(1,2*i-1) * arrow_length], [pts(2,i), pts(2,i) + fc_edges(2,2*i-1) * arrow_length], 'b-');
                plot([pts(1,i), pts(1,i) + fc_edges(1,2*i) * arrow_length], [pts(2,i), pts(2,i) + fc_edges(2,2*i) * arrow_length], 'b-');
            end
        end
        
      end
      
      function [flag_cagged, flag_in, flag_on] = CheckForCagingGeometry(obj, pts, finger_radius)
          % This function checks geometric conditions for fingers caing the
          % object.
          % Input: pts: the contact points column vectors.
          % Output: flag_cagged, whether the object is being cagged or not.
          % flag_in: whether the object is inside the caging boundary.
          % flag_on: whether the object is on the caging boundary.
          n_pts = size(pts, 2);
          flag_cagged = false;
          flag_in = false;
          flag_on = false;
          eps_dist = finger_radius / 4;
          pts_tips = pts;
          % For now, let's start with 3 fingers caging a circle. 
          if (n_pts == 3 && strcmp(obj.shape_type, 'circle'))
            % Compute pairwise distance between the contact points.
            dist_pts = pdist(pts_tips');
            cage_tri_edge = 2 * (obj.shape_parameters.radius + sqrt(3)/2.0 * finger_radius);
            %2 * (obj.shape_parameters.radius + sqrt(3)/2.0 * finger_radius)
            % Check in the caging triangle is formed or not.
            %flag_triangle = ((sum(dist_pts <= ...
            %    2 * (obj.shape_parameters.radius + sqrt(3)/2.0 * finger_radius + eps_dist))) == n_pts);
            flag_triangle = (sum(dist_pts <= cage_tri_edge) == n_pts);
            [flag_in, flag_on] = inpolygon(obj.pose(1), obj.pose(2), pts(1,:), pts(2,:));
            flag_cagged = flag_in & flag_triangle;
          end
      end
   end
end