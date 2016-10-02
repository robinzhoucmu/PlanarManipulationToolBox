% This class allows a user to compute a spiral trajectory given a
% time-parameterized pitch function, s(t), a maximum linear velocity, an
% object radius to stop computing the trajectory at, the initial distance
% away from the origin, and the time discretization.
%
% The pitch relates the counter-clockwise tangential velocity with the
% inward normal velocity. If the pitch is 0, the spiral trajectory is a
% circle, and if the pitch is infinite, then the spiral trajectory is a
% straight line inwards.
%
% s(t) needs to be able to take in a vector of times and return the desired
% pitch at all of those times.
%
% Example Use:
%
% % Create object with a constant pitch of 0 (straigt pushing), a maximum velocity of 1, an
% % object radius of 1, an initial distance away of 2, and a time step of
% % 0.1
% obj = PitchCompute(@(t)(0), 1, 1, 2, 0.1);
% 
% % Plot the trajectory
% obj.plot();
%
% % Get the position and velocity at some times
% [pp,vv] = obj.get_pos_and_vel([0.05, 0.15, 0.25, 0.35, 0.45]');


classdef PitchCompute
    properties (Constant)
       MAX_STEPS = 1000;
       % If the pitch is greater than MAX_PITCH, we just treat it as pure 
       % rotation.
       MAX_PITCH = 2 * pi;
    end
    properties
      % User Specified
      PitchFun % s(t). Given a vector of times, return a vector of desired pitches
      MAX_VEL % The velocity to travel at along the curve
      OBJ_RAD % The radius of the object to stop the trajectory at
      DT % The time step to compute trajectory points at
      INITIAL_R % The initial distance away from the origin to start at
      MAX_TIME % Maximum travelling time.
      
      % Computed
      t % The time steps where the trajectory is computed by solving an ode
      p % The [x y] positions where the trajectory is computed
      v % The [dx/dt dy/dt] velocities where the trajectory is computed
      interp_p % An interpolation polynomial that uses the above to compute a new trajectory pose
   end
   methods (Access = public)
      % Main Constructor. Specify the pitch function, maximum velocity,
      % object radius, initial distance away, and the time step. 
      %
      % Computes the spiral trajectory using ode45 and creates an
      % interpolating polynomial that can be used for computing the
      % trajectory at arbitrary time points
      function obj = PitchCompute(pitch_fun, max_vel, obj_rad, initial_r, dt, max_time)
          obj.PitchFun = pitch_fun;
          obj.MAX_VEL = max_vel;
          obj.OBJ_RAD = obj_rad;
          obj.DT = dt;
          obj.INITIAL_R = initial_r;
          if (nargin == 5)
            max_time = obj.MAX_STEPS * obj.DT;
          end
          obj.MAX_TIME = max_time;
          opts=odeset('Events',@obj.stop_event_radius);
          [time,position] = ode45(@obj.spirofun, 0:obj.DT:obj.MAX_TIME, [obj.INITIAL_R;0], opts);
          
          obj.t = time;
          % Note that ode returns N*2 row vectors.
          obj.p = position';
          obj.v = obj.compute_vels(time, obj.p);
          obj.interp_p = pchipd(obj.t, obj.p, obj.v);
      end
      
      % Returns the position and velocity at desired time points 't'.
      % Position and velocity will both be N x 2 matrices, where N is the
      % number of time points.
      function [pp,vv] = get_pos_and_vel(obj,t)
          pp = ppval(obj.interp_p, t)';
          vv = obj.compute_vels(t, pp')';
      end
 
      % Plot the path the spiral takes as computed by ode45
      function plot(obj, num_fingers)
        if (nargin < 2)
            num_fingers = 1;
        end
        figure;
        hold on;
        for i = 1:1:num_fingers
            angle = (i-1) * 2*pi / num_fingers;
            R = [cos(angle), -sin(angle);
                sin(angle), cos(angle)];
            pts = R * obj.p;
            plot(pts(1,:), pts(2,:), 'b-', 'LineWidth', 1.75);
        end
        axis equal; 
        axis([-obj.INITIAL_R,obj.INITIAL_R,-obj.INITIAL_R,obj.INITIAL_R]);
       end    
   end
   
   methods (Access = public)
      
      function v = compute_vels(obj, time, position)
          % Get how much rotation versus squeezing.
          pitch = obj.PitchFun(time);
          %if (pitch <= obj.MAX_PITCH) & (time > 4.5 | time < 0.5)
          if (pitch <= obj.MAX_PITCH) %& (time > 4.5 | time < 0.5)
             st = 1.0 / (eps + obj.PitchFun(time));
             x = position(1,:);
             y = position(2,:);
             mags = obj.MAX_VEL  ./(sqrt(1+st.^2) .* sqrt(x.^2 + y.^2));
             v = zeros(size(position));
             v(1,:) = mags .* (-y - x .* st);
             v(2,:) = mags .* (x - st .* y);
          else
            v = obj.compute_vel_pure_rotation(time, position);
%              testout_pitch = 4*pi;
%              st = 1.0 / (eps + testout_pitch);
%              x = position(1,:);
%              y = position(2,:);
%              mags = obj.MAX_VEL  ./(sqrt(1+st.^2) .* sqrt(x.^2 + y.^2));
%              v = zeros(size(position));
%              v(1,:) = mags .* (-y - x .* st);
%              v(2,:) = mags .* (x - st .* y);
          end
      end
      
      function dpdt = spirofun(obj, t,p)
         dpdt = obj.compute_vels(t,p);
      end
      
      function [value,isterminal,direction] = stop_event_radius(obj,t,p)
        value = p(1)^2 + p(2)^2 - obj.OBJ_RAD^2; 
        isterminal = 1;
        direction = -1;
      end
      
      function v = compute_vel_pure_rotation(obj, time, position)
         x = position(1,:);
         y = position(2,:);
         vel_dir = [-y;x];
         vel_dir = bsxfun(@rdivide, vel_dir, sqrt(sum(vel_dir.^2)));
         v = vel_dir * obj.MAX_VEL;
      end
      
      
   end
end

