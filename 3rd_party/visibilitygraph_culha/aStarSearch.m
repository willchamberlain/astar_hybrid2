function asd = aStarSearch(allLines)

global goal start;

nodeSet = createNodes(allLines);

currVert = start;
OSet = findNeiVert(currVert,allLines,nodeSet);
stNode = struct('pt',start,'hn',findSqDistance(start,goal),'gn',0,'bp',[0 0]);
CSet = [stNode];
OSetnew = [];
fail = 0;
while(1)

    if isempty(OSet)
        fail = 1;
        break;
    end
    
    
    [ndBest,OSet] = fetchBest(OSet); 
    CSet = [CSet; ndBest];
    if sum(ndBest.pt == goal) == 2
        fail = 0;
        break;
    end
    
    newNei = findNeiVert(ndBest.pt,allLines,nodeSet);
    newNei = removeCSet(newNei,CSet);
    
    if not(isempty(newNei))
        
        sizeNew = size(newNei,1);     
        for i = 1:sizeNew
            node = newNei(i);
            node.gn = node.gn + ndBest.gn;
            
            OSet = [OSet;node];
            
            if ndBest.gn + findSqDistance(ndBest.pt,node.pt) < node.gn
%                 node.gn = node.gn + ndBest.gn;
                node.bp = ndBest;
            end
                
                
        end
        
    end
    
end

if not(fail)
    lines = backTrack(ndBest,stNode,CSet);
    asd = lines;
else
    
end


end