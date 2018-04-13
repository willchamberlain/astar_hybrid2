% Plot data - a line from (1,1) to (10,10).
h=plot(1:10, 'bs-')
grid on;
axis equal;
xlim([0 11]);
ylim([0 11]);
datacursormode on;
% Enlarge figure to full screen.
screenSize = get(0,'ScreenSize')
set(gcf, 'units','pixels','outerposition', screenSize);
% Ask user to click on a point.
uiwait(msgbox('Click near any data point'));
% Print the x,y coordinates - will be in plot coordinates
[x,y] = ginput(1) % Will be close to 5,5 but not exactly.
% Mark where they clicked with a cross.
hold on;
plot(x,y, 'r+', 'MarkerSize', 20, 'LineWidth', 3);
% Print the coordinate, but this time in figure space.
% Coordinates will be way different, like 267, 196 instead of 5,5.
cpFigure = get(gcf, 'CurrentPoint')
cpAxis = get(gca, 'CurrentPoint')
% Print coordinates on the plot.
label = sprintf('(%.1f, %.1f) = (%.1f, %.1f) in figure space', x, y, cpFigure(1), cpFigure(2));
text(x+.2, y, label);
% Tell use what ginput, cpFigure, and cpAxis are.
message = sprintf('ginput = (%.3f, %.3f)\nCP Axis = [%.3f, %.3f\n              %.3f, %.3f]\nCP Figure = (%.3f, %.3f)\n',...
	x, y, cpAxis(1,1), cpAxis(1,2), cpAxis(2,1), cpAxis(2,2), cpFigure(1), cpFigure(2));
uiwait(msgbox(message));
% Retrieve the x and y data from the plot
xdata = get(h, 'xdata')
ydata = get(h, 'ydata')
% Scan the actual ploted points, figuring out which one comes closest to 5,5
distances = sqrt((x-xdata).^2+(y-ydata).^2)
[minValue minIndex] = min(distances)
% Print the distances next to each data point
for k = 1 : length(xdata)
	label = sprintf('D = %.2f', distances(k));
	text(xdata(k)+.2, ydata(k), label, 'FontSize', 14);
end
% Draw a line from her point to the closest point.
plot([x xdata(minIndex)], [y, ydata(minIndex)], 'r-');
% Tell her what data point she clicked closest to
message = sprintf('You clicked closest to point (%d, %d)',...
	xdata(minIndex), ydata(minIndex));
helpdlg(message);