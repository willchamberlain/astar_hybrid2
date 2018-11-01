function linesArr = createVisGra()

global configSpace;

confSize = size(configSpace,2);
accLines = [];
% start with lines from obstacle vertex
for i = 1:confSize-2
    obs = configSpace{i};
    obsSize = size(obs,1);
      
    for j = 1:obsSize -1
        % get the vertex j of obstacle i
        vert = obs(j,:);
        for k = 1:confSize -2
            obs2 = configSpace{k};
            obs2Size = size(obs2,1);          
            if i == k % on the same obstacle
                for l = 1:obs2Size -1
                    vert2 = obs2(l,:);
                    
                    if l~=j % refuse same vertex ( point creation )                 
                        % form a line with vert and vert2
                        % x1 y1 x2 y2
                        lin = [vert(1) vert(2); vert2(1) vert2(2)];
                        % accept adjacent vertex
                        if (l == mod(j+1,9)) || (l == (mod(j -1,9) + (j==1)*9)) 
                           linA = [lin(2,:) lin(1,:)];
                           if not(findSame(accLines,linA))
                                accLines = [accLines; [lin(1,:) lin(2,:)]]; 
                           end
                        else
                           [xint,yint] = polyxpoly(lin(:,1),lin(:,2),obs2(:,1),obs2(:,2),'unique');
                            % check if line is a poly diagonal
                            if size(xint,1) >= 2
                                xpts = linspace(lin(1,1),lin(2,1));
                                ypts = linspace(lin(1,2),lin(2,2));
                                ins = inpolygon(xpts,ypts,obs2(:,1),obs2(:,2));
                                % accept if only vertexs are in common
                                if sum(ins) <= 2
                                    linA = [lin(2,:) lin(1,:)];
                                    if not(findSame(accLines,linA))
                                        accLines = [accLines; [lin(1,:) lin(2,:)]]; 
                                    end
                                end
                            end 
                        end                      
                    end                  
                end
            else % other obstacles
                for l = 1:obs2Size -1
                    vert2 = obs2(l,:);
                    
                    % form a line with vert and vert2
                    % x1 y1 x2 y2
                    lin = [vert(1) vert(2); vert2(1) vert2(2)];
                    
                    % first check if line is obs1 diagonal
                    [xint,yint] = polyxpoly(lin(:,1),lin(:,2),obs(:,1),obs(:,2),'unique');
                    if size(xint,1) == 1
                        xpts = linspace(lin(1,1),lin(2,1));
                        ypts = linspace(lin(1,2),lin(2,2));
                        ins = inpolygon(xpts,ypts,obs(:,1),obs(:,2));
                        % accept if vertex is visible to other
                        if sum(ins) <= 2
                            % now check the other obstacle 
                            [xint2,yint2] = polyxpoly(lin(:,1),lin(:,2),obs2(:,1),obs2(:,2),'unique');
                            if size(xint2,1) == 1
                                ins2 = inpolygon(xpts,ypts,obs2(:,1),obs2(:,2)); 
                                if sum(ins2) <= 2
                                    linA = [lin(2,:) lin(1,:)];
                                    if not(findSame(accLines,linA))
                                        accLines = [accLines; [lin(1,:) lin(2,:)]]; 
                                    end                                   
                                end                               
                            end                            
                        end
                    end                     
                end                
            end           
        end       
    end    
end

% now include goal and start
for i = confSize-1:confSize
    obs = configSpace{i};
    
    for j = 1:confSize-2
        obs2 = configSpace{j};
        obs2Size = size(obs2,1); 
        for k = 1:obs2Size
            vert = obs2(k,:);
            
            % line x1 y1 x2 y2
            lin = [obs(1) obs(2); vert(1) vert(2)];
            
            % check visible vertex
            [xint,yint] = polyxpoly(lin(:,1),lin(:,2),obs2(:,1),obs2(:,2),'unique');
            if size(xint,1) <= 1
                xpts = linspace(lin(1,1),lin(2,1));
                ypts = linspace(lin(1,2),lin(2,2));
                ins = inpolygon(xpts,ypts,obs2(:,1),obs2(:,2));
                % accept if vertex is visible
                if sum(ins) <= 2
                    tmpClr = 0;
                    % now consider other obstacles
                    for l = 1:confSize-2
                        if j~=l
                            obs3 = configSpace{l};
                            ins2 = inpolygon(xpts,ypts,obs3(:,1),obs3(:,2));
                            
                            if sum(ins2) <= 2
                                tmpClr = tmpClr + 0;
                            else
                                tmpClr = tmpClr + 1;
                            end                         
                        end
                    end
                    
                    if tmpClr == 0
                        linA = [lin(2,:) lin(1,:)];
                        if not(findSame(accLines,linA))
                            accLines = [accLines; [lin(1,:) lin(2,:)]]; 
                        end  
                    end                             
                end
            end 
        end
        
    end
    
end

linesArr = accLines;

end