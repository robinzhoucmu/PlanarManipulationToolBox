function [h] = DrawEllipsoid(r, X)
maxd = max(abs(X)) * 10;
step = maxd / 100;
x = X(:,1);
y = X(:,2);
z = X(:,3);

[ xx, yy, zz ] = meshgrid( -maxd:step:maxd, -maxd:step:maxd, -maxd:step:maxd);
shape = r(1) * xx.^2 + r(2) * yy.^2 + r(3) * zz.^2 + ...
        r(4) *  xx.* yy + r(5) * xx.* zz + r(6) * yy .* zz;
h = figure;

p = patch(isosurface(xx,yy,zz,shape,1));
set( p, 'FaceColor', 'g','FaceAlpha', 0.5, 'EdgeColor', 'none' );
view(-10, 20);
%view(3);
grid on;
axis equal;
camlight
end

