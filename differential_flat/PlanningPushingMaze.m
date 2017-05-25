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
        % switching cost
        cost_switch
        % polygonal object geometry
        obj_vertices
        % radius of the pusher
        tip_radius
        % assume two points pusher for now.
        finger_width
        
        % Tree_A (grow from start), Tree_B (grow from end) for RRT-Connect. 
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
        
        function [obj] = SetPlan(obj, pose_start, pose_goal, cost_switch, steer_dist)
            obj.pose_start = pose_start;
            obj.pose_goal = pose_goal;
            obj.steer_dist = steer_dist;
            obj.cost_switch = cost_switch;
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
        function [neighbor_ids] = FindKNNEuclidean(obj, p, tree_id, k)
            dists = bsxfun(@minus, Tr_nodes)
        end
        % Find nearest neighbor using Dubins distance given query node
        % (with action) and a list of candidates (from Euclidean
        % K-neighbors) to evaluate.
        function [nn_id] = FindNNDubins(obj, q, candidate_ids, tree_id)
            
        end
        % For a target, steer a bit from q_near to q_new for a given
        % tree_id(which determines movement direction)
        function [q_new] = Steer(obj, q_near, q_target, tree_id)
        
        end
        % For a q_new from the other tree, try to connect the q_near to
        % q_new. 
        function [flag_joint, q_path] = Connect(obj, q_near, q_new, tree_id)
        
        end
        
        function [] = InitializeTrees(obj)
        
        end
        
        function [traj_obj, traj_pusher, action_records] = RRTConnectPlanPath(obj)
        
        end
        
    end
end