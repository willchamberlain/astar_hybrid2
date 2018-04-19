function [ path__ , f_score, g_score , came_from, open_set, closed_set ] = path_planning__astar(cost_map_, start_, goal_)
            map = cost_map_;             start = start_;            goal = goal_; 
            path__ = [];
            if isempty(map); map = zeros(60,60);  end;  %  all-zeros :  no obstacles
            if isempty(start); start = int32([ 55 50 ] ); end;
            if isempty(goal); goal = int32([ 2  4 ]); end;
            
            
            closed_set = []    ;  %zeros(size(map), 'uint32')  ;  % as a map: 1 is closed, zero is not-closed
            
            open_set = []    ;  %zeros(size(map), 'int32')  ;  % as a map: 1 is in-the-open-set , zero is not-in-the-open_set
            open_set( size(open_set,1)+1 , : ) = start    ;
            
            % For each node, which node it can most efficiently be reached from
            came_from = zeros(2,size(map,1),size(map,2), 'uint32')  ;
            visited = zeros(size(map,1),size(map,2), 'uint32')  ;
            
            % For each node, the cost of getting from the start node to that node.
            g_score = inf( size(map) )    ;
            g_score(  open_set(1,1),open_set(1,2)  ) = 0    ;
            
            % For each node, the total cost of getting from the start node to the goal 
            %   by passing by that node. That value is partly known, partly heuristic.
            f_score = inf( size(map) )    ;
%             f_score(start(1),start(2)) = heuristic_cost_estimate(start, goal)  ;            
            f_score(  open_set(1,1),open_set(1,2)  ) = path_planning__manhattan_distance(start,goal)    ;            
            
            while sum(sum(open_set>0))
                
                f_score_lowest = inf;    open_set_index = -1;
                for ii_ = 1:size(open_set,1)
                    if f_score(open_set(ii_,1), open_set(ii_,2)) < f_score_lowest
                        open_set_index = ii_    ;
                        f_score_lowest = f_score(open_set(ii_,1), open_set(ii_,2))    ;
                    end
                end
                current_node = [ open_set(open_set_index,1), open_set(open_set_index,2) ]    ;
                current_row_ = open_set(open_set_index,1)    ;    current_col_ = open_set(open_set_index,2)    ; 
                if current_node == goal 
                    disp( 'current_node == goal' )
                    return
                end
                open_set(open_set_index,:) = []    ;
                closed_set( size( closed_set,1)+1 , : ) = current_node    ;
                
                
                for ii_row_ = current_node(1)-1:current_node(1)+1  
                    for jj_col_ = current_node(2)-1:current_node(2)+1  %     for each neighbor of current
                        if ii_row_ < 1 || ii_row_ >= size(map,1) || jj_col_ < 1 || jj_col_ >= size(map,2) ;  continue  ;   end 
                        if 0 == ii_row_ && 0 == jj_col_  ;  continue  ;   end
                        if sum ( closed_set (  : , 1 ) == ii_row_  &  closed_set (  : , 2 ) == jj_col_ )  > 0 ;  continue  ;   end      % if neighbor in closedSet
                        if sum ( open_set (  : , 1 ) == ii_row_  &  open_set (  : , 2 ) == jj_col_ )  < 1;          %  if neighbor not in openSet	
                            open_set( size(open_set,1)+1 , : ) = [ ii_row_ , jj_col_ ] ;                            
                        end
                        tentative_gScore ...
                            = g_score( current_node(1) , current_node(2) )  ... 
                            + 1 ....
                            + cost_map_( ii_row_ , jj_col_ )   ;   %  tentative_gScore := gScore[current] + dist_between(current, neighbor)
                        %tentative_gScore = tentative_gScore + path_planning__manhattan_distance(current_node , [ ii_row_, jj_col_] )    ;
                        if tentative_gScore > g_score( ii_row_, jj_col_ )  ;    continue  ;   end      
                        
                        came_from( : , ii_row_ , jj_col_ ) =   current_node   ;
                        g_score( ii_row_ , jj_col_ ) = tentative_gScore    ;
                        f_score( ii_row_ , jj_col_ ) = g_score( ii_row_ , jj_col_ ) + path_planning__manhattan_distance( [ ii_row_, jj_col_] , goal )    ;
                    end
                end                
                
            end   
end
                
                