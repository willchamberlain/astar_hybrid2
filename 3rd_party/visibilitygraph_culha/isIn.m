function res = isIn(ex,sp)

if isempty(sp)
    res = 1;
else
    t1 = (sp(:,1) - ex(1)== 0);
    t2 = (sp(:,2) - ex(2)== 0);
    i = find((t1 + t2) == 2,1);

    if isempty(i)
        res = 1;
    else
        res = 0;
    end
    
end

end
