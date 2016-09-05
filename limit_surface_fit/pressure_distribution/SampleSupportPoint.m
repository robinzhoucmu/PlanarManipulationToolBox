% Sample a certain number of support points around the point of origin
% with a pattern specified in options.
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
% Pts: numPoints(Approx) * 2.

function [Pts] = SampleSupportPoint(numPoints, options)
if nargin == 1
    options.mode = 'rim';
    options.range = 2;
end
% Sample inside a polygon.
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
    
    num_samples = numPoints * (area_en / area_size);
    % Sample more.
    P = rand(3*num_samples, 2);
    P(:,1) = vx_min + P(:,1) * (vx_max - vx_min);
    P(:,2) = vy_min + P(:,2) * (vy_max - vy_min);

    % Rejection sampling.
    InPoly = inpolygon(P(:,1), P(:,2), vx, vy);
    Pts = P(InPoly, :);
    
    % Trim.
    Pts = Pts(1:numPoints, :);
    
% Sample inside a circle.
elseif strcmp(options.mode, 'circle') 
    R = options.range;
    angles = rand(numPoints, 1) * 2 * pi;
    len = rand(numPoints, 1) * R;
    Pts = [len .* cos(angles), len .* sin(angles)];

% Sample on the rim of a circle.
elseif strcmp(options.mode, 'rim')
    R = options.range;
    angles = rand(numPoints, 1) * 2 * pi;
    Pts = [R * cos(angles), R * sin(angles)];
end

Pts = bsxfun(@minus, Pts, mean(Pts,1));
end

