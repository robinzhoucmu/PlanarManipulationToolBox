function r = sunflower_sampling_radius(k,n,b)
    if k>n-b
        r = 1;            % put on the boundary
    else
        r = sqrt(k-1/2)/sqrt(n-(b+1)/2);     % apply square root
    end
end