% F, V: 3*N. 
% Assume V is normalized velocity!
function [] = VisualizeForceVelPairs(F, V, fig_h, mk_size)
if (nargin < 4)
    mk_size = 8;
end
%fig = figure;
hold on;
figure(fig_h);
numF = size(F,2);
numV = size(V,2);
avg_f_norm = mean(sqrt(sum(F.^2)));
l = avg_f_norm * 0.2;
assert(numF == numV);
% Plot Forces as red dots.
plot3(F(1,:), F(2,:), F(3,:), 'Marker', 'o', 'MarkerFaceColor', 'r', 'Markersize', mk_size, 'LineStyle', 'none');
hold on;
V = V * l;
%Plot Velocities as arrows from force points.
if numF <= 100
for i = 1:1:numF
    arrow = quiver3(F(1,i), F(2,i), F(3,i), V(1,i), V(2,i), V(3,i), 'b-', 'LineWidth', 1.5);
%    adjust_quiver_arrowhead_size(arrow, 8.0);
end
end
% Something weird about arrow size.
%quiver3(F(1,:)', F(2,:)', F(3,:)', V(1,:)', V(2,:)', V(3,:)', 'b-');
end

