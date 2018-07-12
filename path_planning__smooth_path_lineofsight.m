%{
https://www.gamasutra.com/view/feature/131505/toward_more_realistic_pathfinding.php?print=1 
checkPoint = starting point of path
currentPoint = next point in path
while (currentPoint->next != NULL)
if Walkable(checkPoint, currentPoint->next)
// Make a straight path between those points:
temp = currentPoint
currentPoint = currentPoint->next
delete temp from the path
else
checkPoint = currentPoint
currentPoint = currentPoint->next
%}
function total_path_smoothed__ = path_planning__smooth_path_lineofsight(total_path_start_to_goal_, cost_map_, varargin)    
    threshold_ = Inf    ;
    skip_ = 1    ;
    for ii_ = 1:max(max(size(varargin)))
        if strcmpi( varargin{ii_} , 'threshold' )
            threshold_ = varargin{ii_+1}    ;
        elseif    strcmpi( varargin{ii_} , 'skip' )
            skip_ = varargin{ii_+1}            ;
        end
    end
    total_path_smoothed__ = total_path_start_to_goal_    ;
    checkfromPointIndex = 1;                                            %  checkPoint = starting point of path
    currentToPointIndex =  checkfromPointIndex + 1;       %  currentPoint = next point in path
    while currentToPointIndex < size(total_path_smoothed__,2)
        %  checkPoint = starting point of path
        checkfromPoint = total_path_smoothed__(:,checkfromPointIndex);
        %  currentPoint = next point in path
        currentToPoint = total_path_smoothed__(:,currentToPointIndex);
        
            %         
            %         % obstacle detection - approximation 
            %         if checkfromPoint(1,1) < currentToPoint(1,1)
            %             x_low = checkfromPoint(1,1); 
            %             x_high = currentToPoint(1,1);
            %         else
            %             x_high = checkfromPoint(1,1); x_low = currentToPoint(1,1);
            %         end
            %         if checkfromPoint(2,1) < currentToPoint(2,1)
            %             y_low = checkfromPoint(2,1); 
            %             y_high = currentToPoint(2,1);
            %         else
            %             y_high = checkfromPoint(2,1); y_low = currentToPoint(2,1);
            %         end
            %         detectionLocality = cost_map_( x_low:x_high , y_low:y_high )    ;
            %         if isinf(threshold_)  
            %             is_walkable = sum(sum(isinf(detectionLocality))) <= 0;
            %         else
            %             is_walkable = sum(sum( detectionLocality(:,:)>threshold_ )) <= 0;
            %         end
            %         
            %             x_low = checkfromPoint(1,1); 
            %             x_high = currentToPoint(1,1);
            %             y_low = checkfromPoint(2,1); 
            %             y_high = currentToPoint(2,1);
            % points_on_line__ = geom__points_on_line(Xstart, Ystart, Xend, Yend)
            %         points_on_line__ = geom__points_on_line_values([63,49] , [75,47] , map_2))  ;
        checkfromPoint_xy = flip(checkfromPoint,1)  ;
        currentToPoint_xy = flip(currentToPoint,1)  ;
        points_on_line_values = test__geom__points_on_line__3( checkfromPoint_xy ,  currentToPoint_xy , cost_map_)  ;
        if isinf(threshold_)  
             is_walkable = sum(sum(isinf(points_on_line_values))) <= 0;
        else
             if sum(sum(size(max( points_on_line_values )))) == 0
                 display('sum(sum(size(max( points_on_line_values )))) == 0')
             end
             is_walkable = max( points_on_line_values ) < threshold_ ;
         end
        
        if is_walkable
                node_from_xy = flip( total_path_smoothed__(:,checkfromPointIndex),1)  ;
                node_to_remove_xy = flip(total_path_smoothed__(:,currentToPointIndex),1)  ;
                plot( [node_from_xy(1) node_to_remove_xy(1) ] , [node_from_xy(2) node_to_remove_xy(2)] , 'g')
%             last_node_removed = total_path_smoothed__(:,currentToPointIndex)  ;    
            total_path_smoothed__(:,currentToPointIndex) = []    ;
%             total_path_ignore_(:,currentToPointIndex) = true;
%             currentToPointIndex = currentToPointIndex+1
%             pause
        else
            checkfromPointIndex = currentToPointIndex     ;
            currentToPointIndex =  checkfromPointIndex + skip_     ;
%             pause(0.1)
        end
    end
end

% 