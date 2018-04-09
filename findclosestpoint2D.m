% https://au.mathworks.com/matlabcentral/answers/165916-display-information-about-a-point-by-hovering-over-it-on-a-figure
% by Matt Dash, 2014

function index=findclosestpoint2D(xclick,yclick,datasource)
%this function checks which point in the plotted line "datasource"
%is closest to the point specified by xclick/yclick. It's kind of 
%complicated, but this isn't really what this demo is about...
 
xdata=get(datasource,'xdata');
ydata=get(datasource,'ydata');
 
activegraph=get(datasource,'parent');
 
pos=getpixelposition(activegraph);
xlim=get(activegraph,'xlim');
ylim=get(activegraph,'ylim');
 
%make conversion factors, units to pixels:
xconvert=(xlim(2)-xlim(1))/pos(3);
yconvert=(ylim(2)-ylim(1))/pos(4);
 
Xclick=(xclick-xlim(1))/xconvert;
Yclick=(yclick-ylim(1))/yconvert;
 
Xdata=(xdata-xlim(1))/xconvert;
Ydata=(ydata-ylim(1))/yconvert;
 
Xdiff=Xdata-Xclick;
Ydiff=Ydata-Yclick;
 
distnce=sqrt(Xdiff.^2+Ydiff.^2);
 
index=distnce==min(distnce);
 
index=index(:); %make sure it's a column.
 
if sum(index)>1
    thispoint=find(distnce==min(distnce),1);
    index=false(size(distnce));
    index(thispoint)=true;
end