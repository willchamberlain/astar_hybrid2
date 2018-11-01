function visibilityGraph

% give init map, obstacles and robot
global map robot goal start configSpace;

map = [];
% obs1 = [20,80; 100,80;70,160];
% experimen1 
% obs1 = [100,160; 140, 190; 170, 240; 90, 200; 110 190];
% obs2 = [30,80;60,150; 90,110;100,70;60,100];
% obs3 = [140,20; 160,50; 180,50;170,120;150,120;150,90;130,70];
% robot = [5 15; 10 10; 10 25];
% goal = [100 270];
% start = [40 10];

% experiment 2
% obs1 = [30,70; 70,70; 70,110; 30, 110];
% obs2 = [110,140;150,140; 150,160;110,160];
% obs3 = [100,40; 160,40; 160,90;100,90];
% obs4 = [80,190;110,190;110,230;80,230];
% robot = [5 15; 10 10; 10 25];
% goal = [150 210];
% start = [40 10];
% experiment 3
obs1 = [30,70; 70,70; 70,110; 30, 110];
obs2 = [110,140;150,140; 150,160;110,160];
obs3 = [100,40; 160,40; 160,90;100,90];
obs4 = [80,190;110,190;110,230;80,230];
robot = [10 15; 10 10; 5 25];
goal = [150 210];
start = [40 10];

map{1} = obs1;
map{2} = obs2;
map{3} = obs3;
map{4} = obs4;



% draws the given map
drawMap();

% implement the star algorithm
configSpace = [];
configSpace = starAlgorithm();

figure(2)
drawConfigMap();
hold on;

confSize = size(configSpace,2);
configSpace{confSize+1} = start;
configSpace{confSize+2} = goal;

% create visibility graph
lines = createVisGra();
% for i = 1:size(lines,1) 
%     plot([lines(i,1) lines(i,3)],[lines(i,2) lines(i,4)],'b');
% end


% create reduced visibility graph
lines = reduceVisGra(lines);

for i = 1:size(lines,1) 
    plot([lines(i,1) lines(i,3)],[lines(i,2) lines(i,4)],'r');
end

lines = aStarSearch(lines);
for i = 1:size(lines,1) 
    plot([lines(i,1) lines(i,3)],[lines(i,2) lines(i,4)],'g','LineWidth',2);
end


end