% https://au.mathworks.com/matlabcentral/answers/165916-display-information-about-a-point-by-hovering-over-it-on-a-figure
% by Matt Dash, 2014

function mousemove(src,ev,L,animals)
 
%since this is a figure callback, the first input is the figure handle:
f=src;
 
%like all callbacks, the second input, ev, isn't used. 
 
%determine which object is below the cursor:
obj=hittest(f); %<-- the important line in this demo
 
if obj==L %if over the plot...
    %get cursor coordinates in its axes:
    a=get(L,'parent');
    point=get(a,'currentpoint');
    xclick=point(1,1,1);
    yclick=point(1,2,1);
 
    %determine which point we're over:
    idx=findclosestpoint2D(xclick,yclick,L);
 
    %make a "tool tip" that displays this animal.
    xoffset=5;
    yoffset=2;
 
    delete(findobj(f,'tag','mytooltip')); %delete last tool tip
    text(animals{idx,3}+xoffset,animals{idx,2}+yoffset,animals{idx,1},...
        'backgroundcolor',[1 1 .8],'tag','mytooltip','edgecolor',[0 0 0],...
        'hittest','off');
else
    delete(findobj(f,'tag','mytooltip')); %delete last tool tip
 
end
 