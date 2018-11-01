function newNeiS = removeCSet(newNei,CSet)
% removes the already visited nodes from the new neighboor set
newSize = size(newNei,1);
CSize = size(CSet,1);

newNeiS = [];
for i = 1:newSize
    node = newNei(i);
    vert = node.pt;
    
    clrV = 0;
    
    for j = 1:CSize
        node2 = CSet(j);
        vert2 = node2.pt;
        
        if sum(vert == vert2) == 2
            clrV = clrV + 1;
        else
            clrV = clrV + 0;
        end
    end
    
    if clrV == 0
        newNeiS = [newNeiS;node];
    end
    
end

end