classdef PlanningGraph < handle
    properties
        pose_range_min
        pose_range_max
        % num of grids per dimension
        nd
        % goal state
        pose_goal
        % all push actions (dubins)
        all_push_actions
        % num of way points for dubins path
        num_steps
        % cost of switching actions
        cost_switch
        % the pose for each node
        pose_nodes
        % precursor index matrix to encode the shortest path from every
        % node to the goal.
        prev_node
        % shortest distance from each node to the goal.
        dists
    end
    methods (Access = public)
        function obj = PlanningGraph(all_push_actions, pose_goal, num_steps)
            obj.all_push_actions = all_push_actions;
            obj.pose_goal = pose_goal;
            if (nargin < 3)
                num_steps = 50;
            end
            obj.num_steps = num_steps;
        end
        
        function [] = SetRangeAndDiscretization(obj, r_min, r_max, nd)
            obj.pose_range_min = r_min;
            obj.pose_range_max = r_max;
            obj.nd = nd;
        end
        
        % Grid the state space and construct nodes.
        function [] = ConstructNodes(obj)
            % uniform grid
%             x = linspace(obj.pose_range_min(1), obj.pose_range_max(1), obj.nd);
%             y = linspace(obj.pose_range_min(2), obj.pose_range_max(2), obj.nd);
%             theta = linspace(obj.pose_range_min(3), obj.pose_range_max(3), obj.nd);
            % random sampling
            x = bsxfun(@plus,obj.pose_range_min(1), rand(obj.nd^3, 1) * (obj.pose_range_max(1) - obj.pose_range_min(1)));
            y = bsxfun(@plus,obj.pose_range_min(2), rand(obj.nd^3, 1) * (obj.pose_range_max(2) - obj.pose_range_min(2)));
            theta = bsxfun(@plus,obj.pose_range_min(3), rand(obj.nd^3, 1) * (obj.pose_range_max(3) - obj.pose_range_min(3)));
            obj.pose_nodes = [x';y';theta'];
            %[X, Y, Theta] = meshgrid(x, y, theta);
            %obj.pose_nodes = zeros(3, length(X(:)));
            %obj.pose_nodes = [X(:)'; Y(:)'; Theta(:)'];

        end
        
        % Construct the graph and run dijkastra.
        function [] = ConstructGraph(obj, cost_switch)
            obj.cost_switch = cost_switch;
            obj.ConstructNodes();
            num_nodes = size(obj.pose_nodes, 2);
            num_actions = length(obj.all_push_actions);
            num_expanded_nodes = num_actions * num_nodes;
            obj.dists = -1 * ones(num_expanded_nodes, 1);
            obj.prev_node = -1 * ones(num_expanded_nodes, 1);
            % Initialize distances per node per action.
            for i = 1:1:num_nodes
                for j = 1:1:num_actions
                    obj.dists((i-1) * num_actions + j) = ...
                        obj.all_push_actions{j}.GetDubinsPathLength(obj.pose_nodes(:, i), obj.pose_goal, obj.num_steps);
                end
            end
            % Dijkstra process to compute shortest path
            mark = zeros(num_expanded_nodes, 1);
            for i = 1:1:num_expanded_nodes
                i / num_expanded_nodes
                ids = 1:1:num_expanded_nodes;
                ids_unmarked = ids(mark == 0);
                [min_dist, min_node_id] = min(obj.dists(mark == 0));
                min_node_id = ids_unmarked(min_node_id);
                mark(min_node_id) = 1;
                min_node_original_id = ceil(min_node_id / num_actions);
                min_node_action_id = min_node_id - (min_node_original_id - 1) * num_actions;
                for j = 1:1:num_expanded_nodes
                    if (mark(j)) 
                        continue;
                    end
                    neighbor_original_id = ceil(j / num_actions);
                    neighbor_action_id = j - (neighbor_original_id - 1) * num_actions;
                    if  neighbor_original_id ~= min_node_original_id
                        new_dist = min_dist + obj.all_push_actions{neighbor_action_id}.GetDubinsPathLength(...
                            obj.pose_nodes(:,neighbor_original_id), obj.pose_nodes(:,min_node_original_id), obj.num_steps);
                        if neighbor_action_id ~= min_node_action_id
                            new_dist = new_dist + obj.cost_switch;
                        end
                        %new_dist, obj.dists(j)
                        if new_dist < obj.dists(j)
                            obj.dists(j) = new_dist;
                            obj.prev_node(j) = min_node_id;
                            %min_node_id, j
                        end
                    end
                end
            end
        end
        
        % Return the way points on the shortest path given a start node id.
        function [way_pts, action_records, tot_path_length] = GetShortestPathInGraph(obj, node_id)
            tot_path_length = obj.dists(node_id);
            num_actions = length(obj.all_push_actions);
            cur_node_id = node_id;
            action_records = [];
            way_pts = [];
            while (cur_node_id ~= -1)
                % Path is reversed from the dijkstra process. 
                cur_node_id_orig = ceil(cur_node_id / num_actions);
                cur_action_id = cur_node_id - (cur_node_id_orig - 1) * num_actions;
                way_pts(:,end+1) = obj.pose_nodes(:, cur_node_id_orig);
                action_records(end+1) = [cur_action_id];
                cur_node_id = obj.prev_node(cur_node_id);
            end
            % add the goal node. 
            way_pts(:,end + 1) = obj.pose_goal;
        end
        
        function [way_pts, action_records, min_path_length] = QueryNewStartPose(obj, q_start)
            num_actions = length(obj.all_push_actions);
            min_path_length = 1e+9;
            best_nxt_pt_id = -1;
            best_nxt_action_id = -1;
            for i = 1:1:num_actions
                % check direct path to goal 
                dist_direct_goal = obj.all_push_actions{i}.GetDubinsPathLength(q_start, obj.pose_goal, obj.num_steps);
                if (dist_direct_goal < min_path_length)
                    min_path_length = dist_direct_goal;
                    best_nxt_pt_id = -1;
                    best_nxt_action_id = i;
                end
                % scan through all nodes inside the graph as hopping point.
                for v = 1:1:length(obj.dists)
                    nxt_node_id = ceil(v / num_actions);
                    nxt_action_id = v - (nxt_node_id - 1) * num_actions;
                    if (obj.dists(v) < min_path_length)
                        e_dist = obj.all_push_actions{i}.GetDubinsPathLength(q_start, obj.pose_nodes(:,nxt_node_id), obj.num_steps);
                        if nxt_action_id ~= i
                            e_dist = e_dist + obj.cost_switch;
                        end
                        if (e_dist + obj.dists(v) < min_path_length)
                            min_path_length = e_dist + obj.dists(v);
                            best_nxt_pt_id = v;
                            best_nxt_action_id = i;
                        end
                    end
                end
            end
            % If direct path to goal is the shortest
            if (best_nxt_pt_id == -1)
                way_pts = [q_start, obj.pose_goal];
                action_records = best_nxt_action_id;
            else % hopping through nodes in the graph
                [way_pts_intermediate, action_records_intermediate] = obj.GetShortestPathInGraph(best_nxt_pt_id);
                way_pts = [q_start, way_pts_intermediate];
                action_records = [best_nxt_action_id, action_records_intermediate];
            end
        end
        
        % Get a dense sampling of poses along the given way points and action
        % types with specified sampling rate.
        function [traj_obj, traj_pusher, action_ids] = GetCompleteObjectHandPath(obj, way_pts, action_records, ns_seg)
            if nargin < 4
                ns_seg = obj.num_steps;
            end
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
        end
        
        % Visualize a path. 
        function [h] = VisualizePlannedPath(obj, pushobj, hand_two_finger, way_pts, action_records)
            %h = figure;
            num_pushes = length(action_records);
            num_steps_draw = 20;
            width_finger = 24/ 1000;
            for ind_seg = 1:1:num_pushes
                h = figure;
                q_start = way_pts(:, ind_seg);
                q_end = way_pts(:, ind_seg + 1);
                [traj_localframe, traj_pusherframe] = ...
                    obj.all_push_actions{action_records(ind_seg)}.PlanDubinsPath(q_start, q_end, num_steps_draw);
                x = traj_localframe(1,:);
                y = traj_localframe(2,:);
                theta = traj_localframe(3,:);
                num_rec_configs = length(x);
                hold on;
                seg_size = 2;
                for i = 1:1:num_rec_configs
                    if mod(i, seg_size) == 1 || i == num_rec_configs
                    % Plot the object.
                        plot(x(i), y(i), 'b+');
                        obj_pose = [x(i);y(i);theta(i)];
                        vertices = SE2Algebra.GetPointsInGlobalFrame(pushobj.shape_vertices, obj_pose);
                        vertices(:,end+1) = vertices(:,1);
                        if i == 1
                            c= 'k';
                        elseif i == num_rec_configs
                            c = 'b';
                        else
                            c = 'r';
                        end
                        plot(vertices(1,:), vertices(2,:), '-', 'Color', c);
                         hand_two_finger.q = zeros(4,1);
                         hand_two_finger.q(1:3) = traj_pusherframe(:,i);
                         hand_two_finger.q(3) = hand_two_finger.q(3) + pi/2.0;
                         hand_two_finger.q(4) = width_finger;

                         hold on;
                         hand_two_finger.Draw(gcf);
                    end
                end
                axis equal;
            end
        end
        
        
        
    end
end