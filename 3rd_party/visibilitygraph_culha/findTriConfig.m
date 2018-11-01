function configSp = findTriConfig(tri)

global robot;

configSp = [];

% correct given tri poly
[x y] = poly2ccw(tri(:,1),tri(:,2));
tri = [x y];

for i = 1:3
   for j = 1:3
       
       % find the normal of robot
       nr = findNorm(robot(j,:),robot(mod(j,3)+1,:));
       
       case1 = (tri(mod(i+2,3) + (i==1)*3 ,:) - tri(i,:))*nr' >= 0;
       case2 = (tri(mod(i,3)+1,:) - tri(i,:))*nr' >= 0;
       
       if case1 && case2 % if applicability condition holds add vertices
           
           % add extreme cases
           
            ex1 = tri(i,:)-( robot(j,:) - robot(3,:) );
            
            if isIn(ex1,configSp)
                configSp = [configSp; ex1];
            end
                        
            ex2 = tri(i,:)-( robot(mod(j,3) + 1,:) - robot(3,:) );
            
            if isIn(ex2,configSp)
                configSp = [configSp; ex2];
            end
           
       end
       
       % find normal of osbtacle edge
       no = findNorm(tri(i,:),tri(mod(i,3)+1,:));
       
       case1 = (robot(mod(j+2,3) + (j == 1)*3,:)- robot(j,:))*no' >= 0;
       case2 = ( robot(mod(j,3)+1,:) - robot(j,:) )*no' >= 0;
       
        if case1 && case2
           
            ex1 = tri(i,:)-(robot(j,:)-robot(3,:));
           
            if isIn(ex1,configSp)
                configSp = [configSp; ex1];
            end
            
           ex2 = tri(mod(i,3)+1,:)-(robot(j,:)-robot(3,:));
           
           if isIn(ex2,configSp)
                configSp = [configSp; ex2];
           end
            
       end
           
       
   end
    
end

% correct the resulting poly

[row col] = size(configSp);
mid = sum(configSp)/row;
ang = atan2(configSp(:,2)-mid(2), configSp(:,1)-mid(1));
ang = mod(ang,2*pi);
[ang srt] = sort(ang);
configSp = [ configSp(srt,1) configSp(srt,2)];


end