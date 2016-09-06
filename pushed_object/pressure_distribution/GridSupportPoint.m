% Grid a certain number of support points around the point of origin
% with a pattern specified in options.
% The algorithm computes an enclosing rectangle and 
% Input:
% options.mode:
%   "rim"
%   "circle"
%   "polygon"

% options.range:
%   radius for circle/rim.

% options.vertices [x,y](n*2)
%   vertices on the polygon, sorted CCW-wise. 

% Output:
% Pts: numPoints(Approx)*2.
function [Pts] = GridSupportPoint(numPoints, options)
if nargin == 1
    options.mode = 'rim';
    options.range = 2;
end
    
if strcmp(options.mode, 'polygon')
    v = options.vertices;
    vx = v(:,1);
    vy = v(:,2);
    area_size = polyarea(vx, vy);
    
    % Find enclosing rectangular.
    vx_max = max(vx);
    vx_min = min(vx);
    vy_max = max(vy);
    vy_min = min(vy);
    area_en = (vx_max - vx_min) * (vy_max - vy_min);
    % Find the number of grids.
    num_samples = numPoints * (area_en / area_size);
    grid_size = sqrt(area_en / num_samples);
    % Form the grids.
    Px(:,1) = vx_min:grid_size:vx_max;
    Py(:,2) = vy_min:grid_size:vy_max;
    [X, Y] = meshgrid(Px(:,1), Py(:,2));
    P(:,1) = X(:);
    P(:,2) = Y(:);
    % Select grids inside the polygon.
    InPoly = inpolygon(P(:,1), P(:,2), vx, vy);
    Pts = P(InPoly, :);
    
elseif strcmp(options.mode, 'circle')
    R = options.range;
    num_samples = numPoints * (4 / pi);
    grid_size = sqrt(4*R*R / num_samples);
    % Form the grids.
    Px(:,1) = -R:grid_size:R;
    Py(:,2) = -R:grid_size:R;
    
    [X, Y] = meshgrid(Px(:,1), Py(:,2));
    P(:,1) = X(:);
    P(:,2) = Y(:);
    % Select grids inside.
    InCircle = sum(P.^2,2) <= R^2;
    Pts = P(InCircle, :);
    
elseif strcmp(options.mode, 'rim')
    R = options.range;
    grid_size = 2*pi / numPoints;
    angles = [0: grid_size: 2*pi]';
    Pts = [R * cos(angles), R * sin(angles)];
end
end

