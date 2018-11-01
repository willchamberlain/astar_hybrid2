function drawMap()

global map robot start goal;

mapSize = size(map,2);


for i=1:mapSize
   obs = map{i};
   patch(obs(:,1),obs(:,2),'r');
end

hold on;
plot(goal(:,1),goal(:,2),'ro');
hold on;
plot(start(:,1),start(:,2),'bo');
hold on;
% place the coordinate to the 3rd vertex of robot
rx=(robot(:,1)-robot(3,1))+start(1);
ry=(robot(:,2)-robot(3,2))+start(2);
line([rx ;rx(1)],[ry; ry(1)],'Color','g');


axis([0 200 -10 300]);


end