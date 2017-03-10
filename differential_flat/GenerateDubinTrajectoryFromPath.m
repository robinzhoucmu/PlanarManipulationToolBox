% Generate interpolated trajectory in flat space.
% Input: z (2*N), total trajectory time and interpolation method.
function [traj_interp] = GenerateDubinTrajectoryFromPath(z, tot_time, interp_method)
if (nargin < 2)
    tot_time = 1.0;
end
if (nargin < 3)
    interp_method = 'spline';
end
traj_interp =  TrajectoryInterp();
num_pts = size(z,2);
t = linspace(0,tot_time, num_pts);
traj_interp.SetInterpMode(interp_method);
traj_interp.SetPositionOverTime(t, z);
traj_interp.GenerateInterpPolynomial();
end