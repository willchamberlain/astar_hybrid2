function resLines = reduceVisGra(lines)
% increase the line length and check whether this new line is
% included in any obstacle, eliminite that line if true

global configSpace;

confSize = size(configSpace,2);
resLines = [];

lineSize = size(lines,1);

for i = 1:lineSize
    % lin = [x1 y1 x2 y2]
    lin = lines(i,:);
    
    % increase the line by 10 units on both size
    newLine = increaseLine(10,lin);
    lnClr = 0;
    
    % create 2 lines out of extended parts
    l1 = [lin(1) lin(2);newLine(1) newLine(2)];
    l2 = [lin(3) lin(4);newLine(3) newLine(4)];
    
    for j = 1:confSize
        obs = configSpace{j};
        
        % check for l1
        xf1 = linspace(l1(1,1),l1(2,1));
        yf1 = linspace(l1(1,2),l1(2,2));
        
        in1 = inpolygon(xf1,yf1,obs(:,1),obs(:,2));
        
        if sum(in1) < 20
           lnClr = lnClr + 0; 
        else
           lnClr = lnClr + 1; 
        end
        
        % check for l2
        xf2 = linspace(l2(1,1),l2(2,1));
        yf2 = linspace(l2(1,2),l2(2,2));
        
        in2 = inpolygon(xf2,yf2,obs(:,1),obs(:,2)); 
        
        if sum(in2) < 20
           lnClr = lnClr + 0; 
        else
           lnClr = lnClr + 1; 
        end
        
    end  
    
    if lnClr == 0
        resLines = [resLines;lines(i,:)];
    end
       
    
end

end
