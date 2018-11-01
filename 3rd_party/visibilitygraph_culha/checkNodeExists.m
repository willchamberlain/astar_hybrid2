function [res2,nodeR] = checkNodeExists(allNodes,node)

sizeNodes = size(allNodes,1);
nodeR = [];
res = 0;
for i = 1:sizeNodes
   currNode = allNodes(i);
   vert = currNode.pt;
   
   if sum( vert == node) == 2
       res = res + 1;
       nodeR = currNode;
   else
       res = res + 0;
   end
             
end

    if res > 0
        res2 = 1;
    else
        res2 = 0;
        nodeR = [];
    end


end