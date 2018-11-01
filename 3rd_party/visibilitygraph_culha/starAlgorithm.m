function confSpMap = starAlgorithm()

global map;

confSp = [];
confSpMap = [];

mapSize = size(map,2);

for i = 1: mapSize
    obs = map{i};
    obsx = obs(:,1);
    obsy = obs(:,2);

    % check  cw polygon
    if not(ispolycw(obsx,obsy))
        [obsx obsy ]=poly2cw(obsx,obsy);
    end
    
  % triangulate the obstacle for convex convention
   %tri = delaunay(obsx,obsy,{'Qt','Qbb','Qc','Qz'});
   tri = delaunay(obsx,obsy);
   
   % eliminate inner triangles
   tris = size(tri,1);
   tempObs = zeros(tris,3);
   index = 1;
    for j = 1: tris
        % find intersection with obstacle and triangle vertices
        [xint, yint] = polybool('and',obsx,obsy,obsx(tri(j,:)),obsy(tri(j,:)));
        
        if size(xint,1) > 0
            tempObs(index,:) = tri(j,:);
            index = index + 1;
        end
    end
   % remove zeros
   triObsT = sum(tempObs');
    % triObs are triangular obstacles
   triObs = tempObs(1:size(triObsT(triObsT>0),2),:);
   
   % find config spaces of triangles
    triSp = [];
    for k = 1: size(triObs,1)
        
        trix = obsx(triObs(k,:));
        triy = obsy(triObs(k,:));
                
        triSp = findTriConfig([trix triy]);
        
        % join the triangle based config spaces
        if k == 1
            confSp = triSp;       
        else
           [joinx joiny] = polybool('or', confSp(:,1),confSp(:,2),triSp(:,1),triSp(:,2));
           confSp = [joinx joiny];
        end
        
            figure(3)
%            patch(triSp(:,1),triSp(:,2),'c');
           hold on;
           patch(trix,triy,'r');
            
           axis([0 200 -10 300])

        
    end
   
  confSpMap{i} = confSp;  

end



end