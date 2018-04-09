% https://au.mathworks.com/matlabcentral/answers/165916-display-information-about-a-point-by-hovering-over-it-on-a-figure
% by Matt Dash, 2014

function mouseovertest
 
f=figure('renderer','painters');
a=axes;
 
%some animal data:
%name   speed   weight
animals={'Peregrine falcon',200.00,4;
'Cheetah',70.00,110;
'Pronghorn antelope',61.00,60;
'Lion',50.00,150;
'Thomson''s gazelle',50.00,170;
'Wildebeest',50.00,250;
'Quarter horse',47.50,1000;
'Cape hunting dog',45.00,40;
'Elk',45.00,300;
'Coyote',43.00,50;
'Gray fox',42.00,15;
'Hyena',40.00,45;
'Ostrich',40.00,200};
 
%plot the data
L=line(cell2mat(animals(:,3)),cell2mat(animals(:,2)),'marker','o','markersize',10,...
    'markerfacecolor',[.3 .6 1],'markeredgecolor',[0 0 0],'linestyle','none');
xlabel('Weight (lbs')
ylabel('Speed (mph)');
 
%apply mouse motion function
set(f,'windowbuttonmotionfcn',{@mousemove,L,animals});
 