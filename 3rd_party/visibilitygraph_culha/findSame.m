function res = findSame(allSet,ex)

sizeAll = size(allSet,1);
tmp = 0;
for i = 1:sizeAll
   rw = allSet(i,:);
   if rw == ex
       tmp = tmp + 1;
   else
       tmp = tmp + 0;
   end
end

if tmp>0
    res = 1;
else
    res = 0;
end
end