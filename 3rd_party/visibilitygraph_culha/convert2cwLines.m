function resLines = convert2cwLines(lin)

linSize = size(lin,1);
resLines = lin;

for i = 1:linSize
    
    crLin = lin(i,:);
    crlx = [crLin(1);crLin(3)];
    crly = [crLin(2);crLin(4)];
    
    [crlx crly ]=poly2cw(crlx,crly);
    
    resLines(i,:) = [crlx(1) crly(1) crlx(2) crly(2)];
    
end

end