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
    end
    properties
      % User Specified
      PitchFun % s(t). Given a vector of times, return a vector of desired pitches
      MAX_VEL % The velocity to travel at along the curve
      OBJ_RAD % The radius of the object to stop the trajectory at
      DT % The time step to compute trajectory points at
      INITIAL_R % The initial distance away from the origin to start at
      
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
      function obj = PitchCompute(pitch_fun, max_vel, obj_rad, initial_r, dt)
          obj.PitchFun = pitch_fun;
          obj.MAX_VEL = max_vel;
          obj.OBJ_RAD = obj_rad;
          obj.DT = dt;
          obj.INITIAL_R = initial_r;
          
          opts=odeset('Events',@obj.radius_size_event);
          [time,position] = ode45(@obj.spirofun, 0:obj.DT:obj.DT*(obj.MAX_STEPS-1), [obj.INITIAL_R;0], opts);
          obj.t = time;
          obj.p = position;
          obj.v = obj.compute_vels(time, position);
          obj.interp_p = pchipd(obj.t', obj.p', obj.v');
      end
      
      % Returns the position and velocity at desired time points 't'.
      % Position and velocity will both be N x 2 matrices, where N is the
      % number of time points.
      function [pp,vv] = get_pos_and_vel(obj,t)
          pp = ppval(obj.interp_p, t)';
          vv = obj.compute_vels(t, pp')';
      end
 
      % Plot the path the spiral takes as computed by ode45
      function plot(obj)
        figure(1);clf;plot(obj.p(:,1), obj.p(:,2));
        axis equal; axis([-obj.INITIAL_R,obj.INITIAL_R,-obj.INITIAL_R,obj.INITIAL_R]);
      end    
   end
   
   methods (Access = public)
      
      function v = compute_vels(obj, time, position)
          st = 1.0 / (eps + obj.PitchFun(time));
          x = position(1,:);
          y = position(2,:);
          mags = obj.MAX_VEL  ./(sqrt(1+st.^2) .* sqrt(x.^2 + y.^2));
          v = zeros(size(position));
          v(1,:) = mags .* (-y - x .* st);
          v(2,:) = mags .* (x - st .* y);
      end
      
      function dpdt = spirofun(obj, t,p)
        dpdt = obj.compute_vels(t,p);
      end
      
      function [value,isterminal,direction] = radius_size_event(obj,t,p)
        value = p(1)^2 + p(2)^2 - obj.OBJ_RAD^2;
        isterminal = 1;
        direction = -1;
      end
   end
end

function pp = pchipd(x,y,d,xx)
%PCHIPD  Piecewise Cubic Hermite Interpolating Polynomial with Derivatives.
%   PP = PCHIPD(X,Y,D) provides the piecewise cubic polynomial which
%   interpolates values Y and derivatives D at the sites X.  This is meant
%   to augment the built-in Matlab function PCHIP, which does not allow the
%   user to specify derivatives.
%  
%   X must be a vector.
%
%   If Y and D are vectors, then Y(i) and D(i) are the value and derivative
%   to be matched at X(i).
%
%   If Y and D are matrices, then size(Y,2) == size(D,2) == length(X).
%   Also, size(Y,1) == size(D,1).  Use this for interpolating vector valued
%   functions.
%
%   YY = PCHIPD(X,Y,D,XX) is the same as YY = PPVAL(PCHIPD(X,Y,D),XX), thus
%   providing, in YY, the values of the interpolant at XX.
%
%   Example comparing SPLINE, PCHIP, and PCHIPD
%     a = -10;
%     b = 10;
%     x = linspace(a,b,7); 
%     f = @(x) 1./(1+exp(-x));  % logistic function
%     df = @(x) f(x).*(1-f(x)); % derivative of the logistic function
%     t = linspace(a,b,50);
%     r = f(t);
%     p = pchip(x,f(x),t);
%     s = spline(x,f(x),t);
%     q = pchipd(x,f(x),df(x),t);
%     plot(t,r,'k',x,f(x),'o',t,p,'-',t,s,'-.',t,q,'--')
%     legend('true','data','pchip','spline','pchipd',4)
%
%   See also INTERP1, SPLINE, PCHIP, PPVAL, MKPP, UNMKPP.
%

%
% 2010-10-04 (nwh) first version
%

% check inputs
% x must be a vector
if ~isvector(x)
  error('pchipd:input_error','x must be a vector of length > 2.')
end

% get size and orient
n = length(x);
x = x(:);

% make sure x is long enough, we can't construct an interpolating
% polynomial with just one point
if n < 2
  error('pchipd:input_error','x must be a vector of length > 2.')
end

% check y and d
if isvector(y) && isvector(d) && length(y) == n && length(d) == n
  % orient
  y = y(:);
  d = d(:);
  m = 1;
elseif size(y,2) == n && size(d,2) == n && size(y,1) == size(d,1)
  m = size(y,1);
  y = y';
  d = d';
else
  error('pchipd:input_error','y and d must be vectors or matrices of same size with length(x) columns.')
end

% sort breaks & data if needed
if ~issorted(x)
  [x x_ix] = sort(x);
  if m == 1
    y = y(x_ix);
    d = d(x_ix);
  else
    y = y(x_ix,:);
    d = d(x_ix,:);
  end
end

% compute coefficients
coef = zeros(m,n-1,4);
dx = diff(x);
for i = 1:m
  dy = diff(y(:,i));
  coef(i,:,4) = y(1:end-1,i)';
  coef(i,:,3) = d(1:end-1,i)';
  coef(i,:,2) = 3*dy./(dx.^2) - (2*d(1:end-1,i)+d(2:end,i))./dx;
  coef(i,:,1) = -2*dy./(dx.^3) + (d(1:end-1,i)+d(2:end,i))./(dx.^2);
end

% create the piecewise polynomial structure
pp = mkpp(x,coef,m);

% if user requests evaluations
if nargin > 3
  pp = ppval(pp,xx);
end
end