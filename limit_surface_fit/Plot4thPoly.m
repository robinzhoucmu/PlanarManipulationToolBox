% input: coefficients r, data X_{n,3} 
function [h] = Plot4thPoly(r, X)
maxd = max(abs(X)) * 5;
step = maxd / 100;
x = X(:,1);
y = X(:,2);
z = X(:,3);

[ xx, yy, zz ] = meshgrid( -maxd:step:maxd, -maxd:step:maxd, -maxd:step:maxd);
shape = r(1) * xx.^4 + r(2) * yy.^4 + r(3) * zz.^4 + ...
        r(4) *  xx.^3 .* yy + r(5) * xx.^3 .* zz + r(6) * yy.^3 .* xx + ...
        r(7) * yy.^3 .* zz + r(8) * zz.^3 .* xx + r(9) * zz.^3 .* yy + ...
        r(10) *(xx.^2).* (yy.^2) + r(11) *(xx.^2) .* (zz.^2) + r(12) * (yy.^2) .* (zz.^2) + ...      
        r(13) * (xx.^2) .* yy .* zz + r(14) * (yy.^2) .* xx .* zz + r(15) *  (zz.^2) .* xx .* yy;
h = figure;
plot3(x,y,z, 'r.');

p = patch(isosurface(xx,yy,zz,shape,1));
set( p, 'FaceColor', 'g','FaceAlpha', 0.5, 'EdgeColor', 'none' );
view(45, 20);
%view(3);
grid on;
camlight
axis equal;
xlabel('F_x', 'FontSize', 12);
ylabel('F_y', 'FontSize', 12);
zlabel('F_z', 'FontSize', 12);
end

