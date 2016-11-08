function [x,y] = sunflower_sampling(n, alpha)   %  example: n=500, alpha=2
    x = zeros(n,1);
    y = zeros(n,1);
    b = round(alpha*sqrt(n));      % number of boundary points
    phi = (sqrt(5)+1)/2;           % golden ratio
    for k=1:n
        r = sunflower_sampling_radius(k,n,b);
        theta = 2*pi*k/phi^2;
        x(k) = r*cos(theta);
        y(k) = r*sin(theta);
    end
end

