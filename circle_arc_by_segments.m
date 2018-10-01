function h = circle_arc_by_segments(x,y,r, angle_,nsegments)  

if nargin<5
    nsegments=50;
end

old_hold = ishold ; 
hold on
if angle_ > 2*pi+0.00001
    angle_ = mod(angle_,2*pi) ; 
end
th = 0:angle_/nsegments:angle_;
xunit = r * cos(th) + x;
yunit = r * sin(th) + y;
h = plot(xunit, yunit);

if ~old_hold
    hold off
end

end
