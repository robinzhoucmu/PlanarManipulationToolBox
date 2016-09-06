% Given support points and options, assign pressure/weights to each point.
% Input:
% options.mode
%   "uniform"
%   "random"
%   "linear"
% options.coef
%   a,b,c: ax+by+c for linear mode.

% Output:
% Pds: Pressure at each point with sum normalized to 1.

function [Pds] = AssignPressure(Pts, options)
% Default is uniform.
if nargin == 1
    options.mode = 'uniform'
end

NumP = size(Pts, 1);
if (strcmp(options.mode, 'uniform'))
    Pds = ones(NumP, 1);
   
elseif (strcmp(options.mode, 'random'))
    Pds = rand(NumP, 1);

elseif (strcmp(options.mode, 'linear'))
    a = options.coef(1);
    b = options.coef(2);
    c = options.coef(3);
    % Note that points might have negative coordinate, hence linear product
    % might be negative. Take abosulte value instead.
    Pds = abs(a * Pts(:,1) + b * Pts(:,2) + c * ones(NumP, 1));
else
    error('mode is not supported.');
end

% Normalize sum equals 1.
Pds = Pds / sum(Pds);

end

