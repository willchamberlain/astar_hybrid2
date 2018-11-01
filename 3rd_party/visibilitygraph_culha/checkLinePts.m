function res = checkLinePts(line1,line2,pt)
% LINES ARE : L = [ X1 X2 Y1 Y2]

res = 0;
x = pt(1);
y = pt(2);

if x == line1(1) && y == line1(3)
    res = 1;
    return;
end

if x == line1(2) && y == line1(4)
    res = 1;
    return;
end

if x == line2(1) && y == line2(3)
    res = 1;
    return;
end

if x == line2(2) && y == line2(4)
    res = 1;
    return;
end


end