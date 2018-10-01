function h = circle_by_segments(x,y,r,nsegments)  

if nargin<4
    nsegments=50;
end

hold on
th = 0:2*pi/nsegments:2*pi;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
h = plot(xunit, yunit);
hold off

end
