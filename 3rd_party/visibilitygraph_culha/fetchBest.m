function [ndbest,OSetnew] = fetchBest(OSet)
% gets the node that has best fn and removes it from OSet
mini = 1e10;

sizeSet = size(OSet,1);
index = 0;

for i = 1:sizeSet
    
    node1 = OSet(i);
    
    fn = node1.gn + node1.hn;
    
    if fn < mini
       mini = fn;
       ndbest = node1;
       index = i;
    end
    
end
% remove the best node
OSetnew = [OSet(1:index-1);OSet(index+1:sizeSet)];

end