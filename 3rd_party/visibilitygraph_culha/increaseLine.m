function newLine = increaseLine(stepSize,lin)
% oldLine = [x1 y1 x2 y2]
    x1 = lin(1);
    x2 = lin(3);
    y1 = lin(2);
    y2 = lin(4);

    slope = atan2(y2-y1,x2-x1);
%     
%     if x1 < x2
%         if y1 < y2
%             x1 = x1 - stepSize*cos(slope);
%             y1 = y1 - stepSize*sin(slope);
%             x2 = x2 + stepSize*cos(slope);
%             y2 = y2 + stepSize*sin(slope);
%         else
%             x1 = x1 - stepSize*cos(slope);
%             y1 = y1 + stepSize*sin(slope);
%             x2 = x2 + stepSize*cos(slope);
%             y2 = y2 - stepSize*sin(slope);
%         end
%     else
%         if y1 < y2
%             x1 = x1 + stepSize*cos(slope);
%             y1 = y1 - stepSize*sin(slope);
%             x2 = x2 - stepSize*cos(slope);
%             y2 = y2 + stepSize*sin(slope);            
%         else
%             x1 = x1 + stepSize*cos(slope);
%             y1 = y1 + stepSize*sin(slope);
%             x2 = x2 - stepSize*cos(slope);
%             y2 = y2 - stepSize*sin(slope);                        
%         end
%     end
  
%     if slope <0
        x1 = x1 - stepSize*cos(slope);
        y1 = y1 - stepSize*sin(slope);
        
        x2 = x2 + stepSize*cos(slope);
        y2 = y2 + stepSize*sin(slope);
%     else
%         x1 = x1 + stepSize*cos(slope);
%         y1 = y1 + stepSize*sin(slope);
%         
%         x2 = x2 - stepSize*cos(slope);
%         y2 = y2 - stepSize*sin(slope);
%     end
%     clf;
%     figure(3)
%     for i = 1:size(lines,1) 
%         plot([x1 x2],[y1 y2],'r');
%     hold on
%     end
%     for i = 1:size(lines,1) 
%         plot([lin(1) lin(3)],[lin(2) lin(4)],'b');
%     hold on
%     end
%     axis([0 300 0 300]);
    
       newLine = [ x1 y1 x2 y2];
end