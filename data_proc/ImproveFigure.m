function [h] = ImproveFigure(h, flag_grid, flag_line_width)
if (nargin < 2)
    flag_grid = 1;
end
if (nargin < 3)
    flag_line_width = 1;
end
figure(h);
if (flag_grid)
    grid on;
end
set(findall(gcf,'-property','FontName'),'FontName','Times New Roman');
%set(gca, 'xtick', -1:0.5:16);
%set(gca, 'ytick', -360:120:360);
set(gca, 'Linewidth', 1);
x = get(gca, 'XLabel');
y = get(gca, 'YLabel');
set(findall(gcf,'-property','FontSize'),'FontSize', 17.5);

%set(x,'FontSize', 20);
%set(y,'FontSize', 20);
child = get(gca, 'Children');
if (flag_line_width)
    for i=1:size(child,1)
        set(child(i),'Linewidth', 1);
    end
end