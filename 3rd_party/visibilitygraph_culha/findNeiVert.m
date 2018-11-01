function nboor2 = findNeiVert(vert, allLines,nodeSet)

sizeL = size(allLines,1);
nboor = [];

for i = 1: sizeL
    x1 = vert(1);
    y1 = vert(2);
    
    
    currL = allLines(i,:);
    
    if x1 == currL(1) && y1 == currL(2)
        nboor = [nboor; [currL(3) currL(4)]];
    end 
    
    if x1 == currL(3) && y1 == currL(4)
        nboor = [nboor;[currL(1) currL(2)]];
    end
end

sizeN = size(nboor,1);
nboor2 = [];
for i =1:sizeN
    [r,node] = checkNodeExists(nodeSet,nboor(i,:));
    node.gn = findSqDistance(vert,nboor(i,:));
    node.bp = vert;
    nboor2 = [nboor2;node];
       
end

end