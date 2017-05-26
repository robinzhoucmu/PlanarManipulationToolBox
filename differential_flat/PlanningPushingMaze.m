classdef PlanningPushingMaze < handle
    properties
        % assuming a rectangle map: [xmin, xmax; ymin, ymax].
        boundary
        % polygonal obstacles
        obstacle_polygons
        % goal state
        pose_goal
        % start state
        pose_start
        % all push actions (dubins)
        all_push_actions
        % small steering distance (as measured of the dubins curve in flatout space )
        steer_dist
        % nearest euclidean neighbor to look at 
        nn_k
        % switching cost
        cost_switch
        % polygonal object geometry
        obj_vertices
        % radius of the pusher
        tip_radius
        % assume two points pusher for now.
        finger_width
        
        % Each node is (x,y,theta, action_id), the action that ends up
        % reaching at this node. 
        Tr_nodes
        Tr_parent_ids
        num_nodes     
    end
    methods
        function [obj] = PlanningPushingMaze(boundary, obstacle_polygons, all_push_actions) 
            obj.boundary = boundary;
            obj.obstacle_polygons = obstacle_polygons;
            obj.all_push_actions = all_push_actions;
        end
        
        function [obj] = SetPlan(obj, pose_start, pose_goal, cost_switch, steer_dist, nn_k)
            obj.pose_start = pose_start;
            obj.pose_goal = pose_goal;
            obj.steer_dist = steer_dist;
            obj.cost_switch = cost_switch;
            obj.nn_k = nn_k;
        end
        
        function [obj] = SetPolygonObjectAndRoundPusherGeometry(obj, obj_vertices, tip_radius, finger_width)
            obj.obj_vertices = obj_vertices;
            obj.tip_radius = tip_radius;
            obj.finger_width = finger_width;
        end
        
        function [flag_collision] = CheckCollisionGivenObjectAndPusher(obj, pose_obj, pose_pusher)
            % Compute the two finger centers (per column) separated along the x axis of
            % the push frame (note it's different from the two finger hand
            % class where y axis the squeezing direction).
            pt_fingers = bsxfun(@plus, pose_pusher(1:2), ...
                [cos(pose_pusher(3)), -sin(pose_pusher(3)); sin(pose_pusher(3)), cos(pose_pusher(3))] * [-obj.finger_width/2, obj.finger_width/2; 0, 0]);
            buffer = [obj.tip_radius; obj.tip_radius];
            cur_obj_vertices = GetPolygonShapeInWorldFrame(obj.obj_vertices, pose_obj);
            % Check the object is inside the boundary.
            flag_collision = sum( (min(cur_obj_vertices, [], 2) < (obj.boundary(:,1))) | (max(cur_obj_vertices, [], 2) > (obj.boundary(:,2))) );
            if (flag_collision) 
                return; 
            end
            % Check collision between the object polygon and static obstacle polygons. 
            for i = 1:1:length(obj.obstacle_polygons)
                    flag_in_1 = inpolygon(obj.obstacle_polygons{i}(1,:), obj.obstacle_polygons{i}(2,:), ...
                         cur_obj_vertices(1,:), cur_obj_vertices(2,:)); 
                    flag_in_2 = inpolygon(cur_obj_vertices(1,:), cur_obj_vertices(2,:), ...
                        obj.obstacle_polygons{i}(1,:), obj.obstacle_polygons{i}(2,:));
                    % The other case of intersection.
                    pt_intersect = intersectPolylines([cur_obj_vertices, cur_obj_vertices(:,end)]', ...
                        [obj.obstacle_polygons{i}, obj.obstacle_polygons{i}(:,end)]' );
                    flag_intersect = ~isempty(pt_intersect);
                    flag_collision = (flag_intersect || sum(flag_in_2) > 0 || sum(flag_in_1) > 0);
            end
            if (~flag_collision)
                % Check whether the fingers are inside the map boundary.
                flag_collision = sum( (min(pt_fingers, [], 2) < (obj.boundary(:,1) + buffer)) | (max(pt_fingers, [], 2) > (obj.boundary(:,2) - buffer)) );
                if (~flag_collision)
                    for i = 1:1:length(obj.obstacle_polygons)
                        % Check whether the fingers are inside the obstacles.
                         flag_collision = sum(inpolygon(pt_fingers(1,:), pt_fingers(2,:), obj.obstacle_polygons{i}(1,:), obj.obstacle_polygons{i}(2,:))) > 0;
                         % Check the distance of the finger centers to the
                         % obstacles are smaller than the tip radius or not.
                         if (~flag_collision)
                            [dist_fingers] = distancePointPolygon(pt_fingers', obj.obstacle_polygons{i}');
                            flag_collision = sum(dist_fingers <= buffer) > 0;
                            if (flag_collision)
                                break;
                            end
                         end
                    end
                end
            end
        end
        
        % Sequentially check collision along the path, return whether it's
        % collision free and the first way point that is in collision.
        function [flag_collision, ind_first_collision] = CheckCollisionGivenPath(obj, traj_obj, traj_pusher)
            flag_collision = 0;
            ind_first_collision = -1;
            for i = 1:1:length(traj_obj)
                if obj.CheckCollisionGivenObjectAndPusher(traj_obj(:,i), traj_pusher(:,i))
                    flag_collision = 1;
                    ind_first_collision = i;
                    break;
                end
            end
        end
        % Find nearest K neighbors given query point p, this only uses x
        % and y position.
        function [neighbor_ids] = FindKNNEuclidean(obj, p, k)
            dists = sum(bsxfun(@minus, obj.Tr_nodes(1:2,1:obj.num_nodes), p).^2, 1);
            [~,idx] = sort(dists, 'ascend');
            neighbor_ids = idx(1:min(k, obj.num_nodes));
        end
        % Find nearest neighbor using Dubins distance given query node
        % (with action) and a list of candidates (from Euclidean
        % K-neighbors) to evaluate.
        function [nn_id, dubin_dist_min] = FindNNDubins(obj, q, candidate_ids)
            dubin_dist_min = 1e+9;
            nn_id = -1;
            for i = 1:1:length(candidate_ids)
                node_candidate = obj.Tr_nodes(:, candidate_ids(i));
                % If it's the tree extended from the start, then distance is moving from
                % the node on the tree to the query node.
               dist = obj.all_push_actions{q(4)}.GetDubinsPathLength(node_candidate(1:3,:), q(1:3), 10);
               if (q(4) ~= node_candidate(4))
                    dist = dist + obj.cost_switch;
               end
               if (dist < dubin_dist_min)
                    dubin_dist_min = dist;
                    nn_id = i;
               end
            end
        end
        % For a target, steer a bit from q_near to q_new for a given
        % tree_id(which determines movement direction)
        function [q_new, flag_reach, traj_localframe, traj_pusherframe] = Steer(obj, q_near, q_target, num_way_pts)
            [pose_new,flag_reach] = obj.all_push_actions{q_target(4)}.SteerSmallDistance(q_near(1:3), q_target(1:3), obj.steer_dist);
            q_new = [pose_new; q_target(4)];
            if nargin < 4
                num_way_pts = 1;
            end
            if num_way_pts > 1
                [traj_localframe, traj_pusherframe, ~] = obj.all_push_actions{q_target(4)}.PlanDubinsPath(q_near(1:3), pose_new, num_way_pts);
            else 
                traj_localframe = [];
                traj_pusherframe = [];
            end
        end
        
        function [q_rand, flag_target] = SampleRandomNode(obj)
            r = rand();
            action_id =  randi([1 length(obj.all_push_actions)]);
            flag_target = 0;
            if (r <= 0.2)
                q_rand = [obj.pose_goal; action_id];
                flag_target = 1;
            else
                % Sample a node inside the map boundary. Not doing collision checking. 
                r_s = rand(3,1);
                q_rand = [r_s(1) * obj.boundary(1,2)  + (1 - r_s(1)) * obj.boundary(1,1);...
                                r_s(2) * obj.boundary(2,2)  + (1 - r_s(2)) * obj.boundary(2,1);...
                                r_s(3) * 2*pi;...
                                action_id];
            end
        end
        
        function [] = InitializeTrees(obj)
           % Preallocate memory for 2000 nodes.
           obj.Tr_nodes = zeros(4, 2000);
           % Add the start node.
           obj.num_nodes = 0;
           obj.AddNodeToTree([obj.pose_start; 0], -1); 
        end
        
        function [] = AddNodeToTree(obj, node, parent_node_id)
            obj.Tr_nodes(:, obj.num_nodes + 1) = node;
            obj.Tr_parent_ids(obj.num_nodes + 1) = parent_node_id;
            obj.num_nodes = obj.num_nodes + 1;
        end
        
        function [traj_obj, traj_pusher, action_records] = RRTPlanPath(obj)
            obj.InitializeTrees();
            flag_stop = 0;
            h = figure;
            hold on;
            obj.DrawStaticMap(h);
            num_samples = 0;
            while ~flag_stop
                rand_sample_in_collision = 1;
                while (rand_sample_in_collision)
                    [q_rand, flag_target] = obj.SampleRandomNode();
                    pose_pusher_rand = obj.all_push_actions{q_rand(4)}.GetPusherFrameGivenObjLocalFrame(q_rand(1:3));
%                     if (q_rand(1) > 0.23) & (q_rand(1) < 0.27) & (q_rand(2) >0.03) & (q_rand(2) <0.12 )
%                     q_rand, q_new
%                     end
                    [rand_sample_in_collision] = obj.CheckCollisionGivenObjectAndPusher(q_rand(1:3), pose_pusher_rand);
                    %rand_sample_in_collision = 0;
                end
                num_samples = num_samples + 1;
                [neighbor_ids] = obj.FindKNNEuclidean(q_rand(1:2), obj.nn_k);
                [nn_id, dubin_dist_min] = obj.FindNNDubins(q_rand, neighbor_ids); 
                q_near = obj.Tr_nodes(:, nn_id);
                [q_new, flag_reach, traj_localframe, traj_pusherframe] = obj.Steer(q_near, q_rand, 20);
                [flag_collision] = obj.CheckCollisionGivenPath(traj_localframe, traj_pusherframe);
                
                %pose_obj = q_new(1:3);
                %pose_pusher = obj.all_push_actions{q_new(4)}.GetPusherFrameGivenObjLocalFrame(pose_obj);
                %[flag_collision] = obj.CheckCollisionGivenObjectAndPusher(pose_obj, pose_pusher);
                if ~flag_collision
                    obj.AddNodeToTree(q_new, nn_id);
                    obj.num_nodes, num_samples
                    plot(q_rand(1), q_rand(2), '+'); hold on; 
                    plot([q_near(1), q_new(1)], [q_near(2), q_new(2)], 'b-'); hold on; drawnow;
                    if flag_reach && flag_target
                        flag_stop = 1;
                    end
                end
            end
            % get the object, pusher frames and action_ids along the path.
            traj_obj = [];
            traj_pusher = [];
            action_records = [];
            cur_node_id = obj.num_nodes;
            while (cur_node_id~=-1)
                obj_pose = obj.Tr_nodes(1:3, cur_node_id);
                traj_obj(:,end + 1) = obj_pose;
                action_id =  obj.Tr_nodes(4, cur_node_id);
                action_records(end + 1) = action_id;
                if (action_id >= 1)
                    traj_pusher(:,end + 1) = obj.all_push_actions{action_id}.GetPusherFrameGivenObjLocalFrame(obj_pose);
                end
                cur_node_id = obj.Tr_parent_ids(cur_node_id);
            end
            % Reverse direction.
            traj_obj = traj_obj(:, end:-1:1);
            traj_pusher = traj_pusher(:, end:-1:1);
            % The start node does not have action_id.
            action_records = action_records(end-1:-1:1);
        end
        
        function [] = VisualizePath(obj, traj_obj_way_pts, action_records)         
            num_steps_draw = 20;
            num_switches = size(traj_obj_way_pts, 2) - 1;
            color_map = ['b', 'm', 'c', 'r', 'g'];
            for ind = 1:1:num_switches
                h = figure;
                hold on;
                obj.DrawStaticMap(h);
                q_start = traj_obj_way_pts(:, ind);
                q_end = traj_obj_way_pts(:, ind + 1);
                 [traj_localframe, traj_pusherframe] = ...
                    obj.all_push_actions{action_records(ind)}.PlanDubinsPath(q_start, q_end, num_steps_draw);
                x = traj_localframe(1,:);
                y = traj_localframe(2,:);
                theta = traj_localframe(3,:);
                num_rec_configs = length(x);
                hold on;
                seg_size = 2;
                for i = 1:1:num_rec_configs
                    if mod(i, seg_size) == 1 || i == num_rec_configs
                    plot(x(i), y(i), 'b+');
                    obj_pose = [x(i);y(i);theta(i)];
                    vertices = SE2Algebra.GetPointsInGlobalFrame(obj.obj_vertices, obj_pose);
                    vertices(:,end+1) = vertices(:,1);
                    c = color_map(mod(ind, length(color_map)) + 1);
                    plot(vertices(1,:), vertices(2,:), '-', 'Color', c);
                    end
                end
                axis([obj.boundary(1,1) obj.boundary(1,2) obj.boundary(2,1) obj.boundary(2,2)]);
                axis equal;
            end
        
        end
        
        function [] = DrawStaticMap(obj, h)
            figure(h);
            plot([obj.boundary(1,1), obj.boundary(1,1), obj.boundary(1,2), obj.boundary(1,2), obj.boundary(1,1)], ...
                    [obj.boundary(2,1), obj.boundary(2,2), obj.boundary(2,2), obj.boundary(2,1), obj.boundary(2,1)], 'b-');
            hold on;
            plot(obj.pose_goal(1), obj.pose_goal(2), 'r*', 'MarkerSize', 10);
            plot([obj.obstacle_polygons{1}(1,:), obj.obstacle_polygons{1}(1,1)], ...
                [obj.obstacle_polygons{1}(2,:), obj.obstacle_polygons{1}(2,1)], 'r-');
            axis([obj.boundary(1,1) - 0.05, obj.boundary(1,2) + 0.05, obj.boundary(2,1) - 0.05, obj.boundary(2,2) + 0.05]);
            axis equal;
        end
        % For execution on the robot. Add offset to align table center.
        function [traj_obj, traj_pusher, action_ids] = GetPusherExpExecutePath(obj, way_pts, action_records, tc_x, tc_y)
            ns_seg = 30;
            num_actions = length(action_records);
            traj_obj = zeros(3, num_actions * (ns_seg + 1));
            traj_pusher = zeros(3, num_actions * (ns_seg + 1));
            action_ids = zeros(num_actions * (ns_seg + 1), 1);
            for i = 1:1:num_actions
                q_start = way_pts(:, i);
                q_end = way_pts(:, i + 1);
                [traj_obj(:, (i-1) * (ns_seg + 1) + 1: i * (ns_seg + 1)), traj_pusher(:, (i-1) * (ns_seg + 1) + 1: i * (ns_seg + 1))] = ...
                    obj.all_push_actions{action_records(i)}.PlanDubinsPath(q_start, q_end, ns_seg);
                action_ids((i-1) * (ns_seg + 1) + 1: i * (ns_seg + 1)) = action_records(i) * ones(ns_seg+1, 1);  
            end
            trans = [tc_x - obj.boundary(1,2)/2; tc_y - obj.boundary(2,2)/2];
            traj_obj(1:2,:) = bsxfun(@plus, trans, traj_obj(1:2, :));
            traj_pusher(1:2, :) = bsxfun(@plus, trans, traj_pusher(1:2, :));
        end
    end
end