function [ path__ , f_score, g_score , came_from, open_set, closed_set ] = path_planning__astar(cost_map_, start_, goal_)
            map = cost_map_;             start = start_;            goal = goal_; 
            path__ = [];
            if isempty(map); map = zeros(30,30);  end;  %  all-zeros :  no obstacles
            if isempty(start); start = int32([ 29 15 ] ); end;
            if isempty(goal); goal = int32([ 2  4 ]); end;
            
            
            closed_set = zeros(size(map), 'uint32')  ;  % as a map: 1 is closed, zero is not-closed
            
            open_set = zeros(size(map), 'int32')  ;  % as a map: 1 is in-the-open-set , zero is not-in-the-open_set
            open_set(start(1),start(2)) = 1 ;
            
            % For each node, which node it can most efficiently be reached from
            came_from = zeros(2,size(map,1),size(map,2), 'uint32')  ;
            
            % For each node, the cost of getting from the start node to that node.
            g_score = inf( size(map) )  ;
            g_score(start(1),start(2)) = 0  ;
            
            % For each node, the total cost of getting from the start node to the goal
            %   by passing by that node. That value is partly known, partly heuristic.
            f_score = inf( size(map) )  ;
%             f_score(start(1),start(2)) = heuristic_cost_estimate(start, goal)  ;            
            f_score(start(1),start(2)) = norm(double(start-goal),2)  ;            
            
            while sum(sum(open_set>0))
                 [current_row_,current_col_]=find(f_score == min(min(f_score(open_set>0))))  ;
                if size(current_row_,1)>1
                    current_row_=current_row_(1);
                    current_col_ = current_col_(1);
                end
                current_node_index = [current_row_, current_col_]  ;
                if current_node_index == goal
%                     path__ = reconstruct_path(came_from, current_node_index)
                    disp('found goal')
                    return
                end
                
                %  TODO - the edge? - put a border around of lethal cost, and make sure the final path is from a trimmed version
                % close the current node
                closed_set(current_row_, current_col_) = 1  ;                
                
                % open the nodes around the current node that have not already been closed - don't re-visit 
                % TODO -  don't open nodes that have lethal cost map costs 
                % TODO -  don't open nodes that have lethal cost map costs  - could close those
                % TODO -  don't open nodes that have lethal cost map costs  - could not-open those
                % TODO -  what to do with nodes that have 'unknown' cost map costs  - could just plan across them assuming are traversable but medium cost 
                open_set(current_row_-1:current_row_+1 , current_col_-1:current_col_+1)  =  ~closed_set(current_row_-1:current_row_+1 , current_col_-1:current_col_+1)   ;                
                open_set(current_row_, current_col_) = 0  ;   % don't open the current node 
                
                costs_of_neighbours = arrayfun(@(x)(norm(double(x)-double(goal),2)),open_set(current_row_-1:current_row_+1 , current_col_-1:current_col_+1))   ;
                
                costs_of_neighbours_2 = double(~closed_set(current_row_-1:current_row_+1 , current_col_-1:current_col_+1) )  .* costs_of_neighbours    ;
                costs_of_neighbours_3 = double( open_set(current_row_-1:current_row_+1 , current_col_-1:current_col_+1) )  .* costs_of_neighbours_2    ;
                
                % TODO -  cost map costs 
                tentative_gScore = g_score(current_row_, current_col_)+1.0*ones(3)    ; % current node cost plus cost to get to next node
                tentative_gScore(2,2)=inf    ;
                % tentative_gScore < g_score(current_row_-1:current_row_+1 , current_col_-1:current_col_+1)
                
                % ..  trying to do this part as matrices not working  .. may have to go back to a for loop  ..
                [gScore_update_rows, gScore_update_cols] ...
                    = find( tentative_gScore  <  g_score(current_row_-1:current_row_+1 , current_col_-1:current_col_+1) )    ;
                for ii_ = 1:size(gScore_update_rows,1)
                        % gScore[neighbor] := tentative_gScore    
                        g_score( (gScore_update_rows(ii_) + current_row_-2) ,  (gScore_update_cols(ii_) + current_col_-2) ) ...
                            = tentative_gScore(gScore_update_rows(ii_),gScore_update_cols(ii_))    ;  
                        % cameFrom[neighbor] := current
                        came_from(:, (gScore_update_rows(ii_) + current_row_-2) , (gScore_update_cols(ii_) + current_col_-2) ) ...
                            =  current_node_index'     ;   
                        % fScore[neighbor] := gScore[neighbor] + heuristic_cost_estimate(neighbor, goal)        
                        f_score( (gScore_update_rows(ii_) + current_row_-2) , (gScore_update_cols(ii_) + current_col_-2) ) ...
                            = g_score( (gScore_update_rows(ii_) + current_row_-2) , (gScore_update_cols(ii_) + current_col_-2) )  ...
                                + norm( [(gScore_update_rows(ii_) + current_row_-2) ,  (gScore_update_cols(ii_) + current_col_-2) ]  - double(goal),2)  ;    
                end       
                
            end   
end
                
                