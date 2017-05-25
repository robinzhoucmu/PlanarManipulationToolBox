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
        function [obj] = PlanningPushMaze(boundary, obstacle_polygons, all_push_actions) 
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
            buffer = [obj.obj.tip_radius; obj.tip_radius];
            cur_obj_vertices = GetPolygonShapeInWorldFrame(obj.obj_vertices, pose_obj);
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
                flag_collision = sum( (min(pt_fingers, [], 2) < (obj.boundary(:,1) + buffer)) | (max(pt_fingers, [], 2) > (obj.boundary(:,1) - buffer)) );
                if (~flag_collision)
                    for i = 1:1:length(obj.obstacle_polygons)
                        % Check whether the fingers are inside the obstacles.
                         flag_collision = sum(inpolygon(pt_fingers(1,:), pt_fingers(2,:), obj.obstacle_polygons{i}(1,:), obj.obstacle_polygons{i}(2,:)));
                         % Check the distance of the finger centers to the
                         % obstacles are smaller than the tip radius or not.
                         if (~flag_collision)
                            [dist_fingers] = distancePointPolygon(pt_fingers', obj.obstacle_polygons{i}');
                            flag_collision = sum(dist_fingers <= buffer);
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
            neighbor_ids = idx(1:k);
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
        function [q_new, flag_reach] = Steer(obj, q_near, q_target)
            [q_new,flag_reach] = obj.all_push_actions{q_target(4)}.SteerSmallDistance(q_near(1:3), q_target(1:3), obj.steer_dist);
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
            InitializeTrees();
            flag_stop = 0;
            while ~flag_stop
                [q_rand, flag_target] = obj.SampleRandomNode();
                [neighbor_ids] = obj.FindKNNEuclidean(q_rand(1:2), obj.nn_k);
                [nn_id, dubin_dist_min] = obj.FindNNDubins(q_rand, neighbor_ids); 
                q_near = obj.Tr_nodes(:, nn_id);
                [q_new, flag_reach] = obj.Steer(q_near, q_rand);
                pose_obj = q_new(1:3);
                pose_pusher = obj.all_push_actions{q_new(4)}.GetPusherFrameGivenObjLocalFrame(pose_obj);
                [flag_collision] = obj.CheckCollisionGivenObjectAndPusher(pose_obj, pose_pusher);
                if ~flag_collision
                    obj.AddNodeToTree(q_new, nn_id);
                end
                if flag_reach && flag_target
                    flag_stop = 1;
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
                traj_pusher(:,end + 1) = obj.all_push_actions{action_id}.GetPusherFrameGivenObjLocalFrame(obj_pose);
                cur_node_id = obj.Tr_parent_ids(cur_node_id);
            end
            % Reverse direction.
            traj_obj = traj_obj(:, end:-1:1);
            traj_pusher = traj_pusher(:, end:-1:1);
            % The start node does not have action_id.
            action_records = action_records(end-1:-1:1);
        end
        
    end
end