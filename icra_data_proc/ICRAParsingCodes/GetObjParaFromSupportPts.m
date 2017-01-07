% Given discrete object 3 support points, compute pressure distribution and
% radius of gyration.

% Input:
% pts: 2*3; in meters. 
% com: 2*1; in meters.
% mass: in kg. 

% Output:
% pds: 3*1; in Newtons.
% pho: radius of gryration w.r.t the point of origin; in meters.

function [pds, pho] = GetObjParaFromSupportPts(pts, com, mass)
%[x1 x2 x3; y1 y2 y3; 1 1 1] * [p1 p2 p3]' = [m*g*cx, m*g*cy, m*g]'; 
A = [pts;ones(1,3)];
g = 9.8;
G = mass * g;
cx = com(1);
cy = com(2);
y = [G*cx; G*cy; G];

% Compute pressure distribution.
pds = A \ y;
% Compute moment of inertial w.r.t to z axis at the point of origin.
Im = sum(pts.^2, 1) * (pds/g);
% Compute radius of gyration.
pho = sqrt(Im / mass); 

end

