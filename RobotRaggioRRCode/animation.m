clear all
clc
load('matlab3.mat', 'Obstacles', 'X', 'Y', 'xD', 'yD', 'delta', 'RR');

%%
plot(xD,yD,'color', [0.5 0.8 0.3, 0.5],'LineStyle',':', 'LineWidth', 1); 

xmin = -30;
xmax = 30;
ymin = -20;
ymax = 30;
markerSizeInAxesUnits = 3;

xlim([xmin, xmax]);
ylim([ymin, ymax]);

hold on

% Plot Obstacles
for i=1:size(Obstacles,2)
    circ1 = nsidedpoly(1000, 'Center', Obstacles(:,i)', 'Radius', delta);
    plot(Obstacles(1,i), Obstacles(2,i),'bo','LineWidth',5);
    plot(circ1, 'FaceColor', 'y')
end

plot(X,Y,'color', [0 0 0.4, 0.5], 'LineStyle',':', 'LineWidth', 1)
axis equal

ax = gca;

h = hgtransform('Parent',ax);
j = hgtransform('Parent',ax);

r = RR; % raggio del robot

pl1 = rectangle('Position', [xD(1)-r, xD(2), 2*r, 2*r],...
                      'Curvature', [1 1],...
                      'EdgeColor', 'k',...
                      'LineWidth', 1,...
                      'LineStyle', '-',...
                      'FaceColor', [0.5 0.8 0.3, 0.5],...
                      'Parent', h);

pl2 = rectangle('Position', [X(1)-r, Y(1)-r, 2*r, 2*r],...
                      'Curvature', [1 1],...
                      'EdgeColor', 'k',...
                      'LineWidth', 1,...
                      'LineStyle', '-',...
                      'FaceColor', [0 0 0.4, 0.5],...
                      'Parent', j);

% Crea gli oggetti di linea per l'effetto scia
trail1 = line('XData', [], 'YData', [], 'Color', [0.5 0.8 0.3, 0.5], 'LineWidth', 2, 'Parent', h);
trail2 = line('XData', [], 'YData', [], 'Color', [0 0 0.4, 0.5], 'LineWidth', 2, 'Parent', j);


hold off


for k = 2:length(xD)
    
    set(pl1, 'Position', [xD(k)-r, yD(k)-r, 2*r, 2*r]);
    set(pl2, 'Position', [X(k)-r, Y(k)-r, 2*r, 2*r]);


    % Aggiungi le posizioni correnti degli oggetti 'o' marker alla fine delle linee di scia
    x_trail1 = [get(trail1, 'XData'), xD(k)];
    y_trail1 = [get(trail1, 'YData'), yD(k)];
    x_trail2 = [get(trail2, 'XData'), X(k)];
    y_trail2 = [get(trail2, 'YData'), Y(k)];
    
    % Aggiorna la posizione delle linee di scia
    set(trail1, 'XData', x_trail1, 'YData', y_trail1);
    set(trail2, 'XData', x_trail2, 'YData', y_trail2);
    
    drawnow
end