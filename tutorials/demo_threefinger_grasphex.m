rng(1);
%% Construct pushobj.
% Construct an equilateral hexagonal shape. 
le = 0.02;
limit_surface_model = 'poly4'; % 'quadratic' or 'poly4'. 
% Inside the function, it assumes uniform pressure. You can easily specify
% your own preferred pressure distribution. 
pushobj = CreateNSidedPolygonPushObject(6, le, limit_surface_model);
% put the object initially somewhere.
pushobj.pose= [le/4;le/4;pi/6]; % Grasp.
%pushobj.pose= [le/4;le/4;pi/3]; % Jam.
%pushobj.pose= [le/4;le/4;pi/4]; % Grasp.
%pushobj.pose= [le/4;le/8;pi/5]; % Jam.

%% Construct hand.
finger_radius = 0.002;
hand_three_finger = ConstructThreeFingersOneDofHand(finger_radius);

%% Specify hand trajectory.
% Set hand start configuration and goal configuration.
q_start = [0; 0; pi/3;  1.5 * le + finger_radius];
q_end = [0; 0; pi/3; le/4 + finger_radius];
% Create way points. 
num_way_q = 25;
dim_q = length(q_start);
waypoints_hand_q = zeros(dim_q, num_way_q);
for i = 1:1:dim_q
    waypoints_hand_q(i,:) = linspace(q_start(i), q_end(i), num_way_q);
end
% Create the hand trajectory using spline.
t_max = 1;
t_q = linspace(0, t_max, num_way_q);
hand_traj_opts.q = waypoints_hand_q;
hand_traj_opts.t = t_q;
hand_traj_opts.interp_mode = 'spline';
hand_traj = HandTraj(hand_traj_opts);

%% Set simulation.
mu = 0.2; % Coefficient of friction between the object and the hand.
sim_inst = ForwardSimulationCombinedStateNewGeometry(pushobj, hand_traj, hand_three_finger, mu);
tic;
display('simulation starts');
sim_results = sim_inst.RollOut();
display('simulation ends');
toc;
%% Plot the results and write to a video.
vd_file = '3finger_grasp_hexnut.avi';
movieObj = VideoWriter(vd_file);
movieObj.FrameRate = 10;
open(movieObj);
num_rec_configs = size(sim_results.obj_configs, 2);
h = figure;
hold on;
seg_size = 2;
for i = 1:1:num_rec_configs
    figure(h); hold on;
    if mod(i, seg_size) == 1
    % Plot the square object.
        plot(sim_results.obj_configs(1, i), sim_results.obj_configs(2,i), 'b+');
        vertices = SE2Algebra.GetPointsInGlobalFrame(pushobj.shape_vertices, sim_results.obj_configs(:,i));
        vertices(:,end+1) = vertices(:,1);
        plot(vertices(1,:), vertices(2,:), 'r-');
        if (i == 1) 
            c = 'k';
        elseif (i + seg_size > num_rec_configs)
            c = 'b';
        else
            c = 'g';
        end
        hand_three_finger.Draw(h, sim_results.hand_configs(:, i), c);
        axis equal;
        ImproveFigure(gcf);
        frame = getframe(gcf);
        writeVideo(movieObj, frame);
    end
end

close(movieObj);