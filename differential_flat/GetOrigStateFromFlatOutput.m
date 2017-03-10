% Input: dt, z(2*N) discrete values per dt. 
% Output: the cartesian mapping. The last z is not mapped.
% a, b: ellipsoid representation of limit surface A = diag(a,a,b). 
% r: the distance 
function [x, y, theta] = GetOrigStateFromFlatOutput(dt, z, a, b, r)
dotz = diff(z, 1, 2) / dt;
theta = atan2(-dotz(1,:), dotz(2,:));
size(theta)
x = z(1,1:end-1) + a/(b*r) * sin(theta);
y = z(2,1:end-1) - a/(b*r) * cos(theta);
end