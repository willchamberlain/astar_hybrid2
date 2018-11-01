function lines = backTrack(ndBest,ndStart,CSet)

cSize = size(CSet,1);

v1 = ndBest.pt;
v2 = ndBest.bp;

lines = [v1 v2];

while(1)
    
    [r,newNode] = checkNodeExists(CSet,v2);
       
    v1 = newNode.pt;
    
    v2 = newNode.bp;
    
    lines = [lines; [v1 v2]];
    
    if  sum(v2 == ndStart.pt ) == 2
        break;
    end
        
    
end






end