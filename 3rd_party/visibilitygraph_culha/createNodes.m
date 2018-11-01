function nodeSet = createNodes(allLines)
% use struct type node
% node -> pt = [x,y]
% node -> hn = heuristic dist = d(pt,goal)
% node -> gn = actual distance travelled so far
% node -> bp = back pointer to previous node point


global goal;

sizeL = size(allLines,1);

nodeSet = [];
for i = 1:sizeL
   
    node1 = allLines(i,1:2);
    [ret, nd] = checkNodeExists(nodeSet,node1);
    if not(ret)
        hn = findSqDistance(node1,goal);
        node1s = struct('pt',node1,'hn',hn,'gn',0,'bp',[0 0]);
        nodeSet = [nodeSet;node1s];
    end
    
    node2 = allLines(i,3:4);
    [ret,nd] = checkNodeExists(nodeSet,node2);
    if not(ret)
        hn = findSqDistance(node2,goal);
        node2s = struct('pt',node2,'hn',hn,'gn',0,'bp',[0 0]);
        nodeSet = [nodeSet;node2s];
    end
    
end

end