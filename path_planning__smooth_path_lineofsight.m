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
    for ii_ = 1:max(max(size(varargin)))
        if strcmpi( varargin{ii_} , 'threshold' )
            threshold_ = varargin{ii_+1}    
        end
    end
    total_path_smoothed__ = total_path_start_to_goal_    ;
    checkPointIndex = 1;                                            %  checkPoint = starting point of path
    currentPointIndex =  checkPointIndex + 1;       %  currentPoint = next point in path
    while currentPointIndex <= size(total_path_smoothed__,2)
        %  checkPoint = starting point of path
        checkPoint = total_path_smoothed__(:,checkPointIndex);
        %  currentPoint = next point in path
        currentPoint = total_path_smoothed__(:,currentPointIndex);
        
        % obstacle detection - approximation 
        if checkPoint(1,1) < currentPoint(1,1)
            x_low = checkPoint(1,1); x_high = currentPoint(1,1);
        else
            x_high = checkPoint(1,1); x_low = currentPoint(1,1);
        end
        if checkPoint(2,1) < currentPoint(2,1)
            y_low = checkPoint(2,1); y_high = currentPoint(2,1);
        else
            y_high = checkPoint(2,1); y_low = currentPoint(2,1);
        end
        detectionLocality = cost_map_( x_low:x_high , y_low:y_high )    ;
        if isinf(threshold_)  
            is_walkable = sum(sum(isinf(detectionLocality))) <= 0;
        else
            is_walkable = sum(sum( detectionLocality(:,:)>threshold_ )) <= 0;
        end
        if is_walkable
            total_path_smoothed__(:,currentPointIndex) = []    ;
        else
            checkPointIndex = currentPointIndex    ;
            currentPointIndex =  checkPointIndex + 1    ;
        end
    end
end