function lines = createLines(incell)

cellSize = size(incell,2);
totalLines = 0;
% find the lines in the map
for i=1:cellSize
    totalLines = totalLines + (size(incell{i},1)-1);
end
% lines [x1 y1 x2 y2]
lines = zeros(totalLines,4);
linesInd = 1;
for i = 1:cellSize
    arrSize = size(incell{i},1);
    for j = 1:(arrSize-1)
        lines(linesInd,:) = [incell{i}(j,1),incell{i}(j,2)...
                             incell{i}(j+1,1),incell{i}(j+1,2)];
        linesInd = linesInd + 1;
    end
end

end