%{
RGB triplets:  [1 1 0]
Easier: HSV to RGB  :  [rout,g,b] = hsv2rgb(hin,s,v)
%}

function path_planning__draw_path_3d( total_path_goal_to_start_ , height_ , varargin )
    pause_on = false;
    for ii_ = 1:max(max(size(varargin)))
        if strcmpi( varargin{ii_} , 'pause' )
            pause_on = true;
        end
    end
hold on
path_nodes = flip( total_path_goal_to_start_ , 1);
for ii_ = 1:size(total_path_goal_to_start_,2) 
    hue = ii_ * 1/size(total_path_goal_to_start_,2) ;
    plot3( path_nodes( 1 , ii_ ) , path_nodes( 2 , ii_ ) , height_(ii_), 's' , 'Color', squeeze( hsv2rgb( 1-hue ,1,1) )' , 'Linewidth', 10 )    
    plot3( path_nodes( 1 , ii_ ) , path_nodes( 2 , ii_ ) , height_(ii_), 's' , 'Color', squeeze( hsv2rgb( 1,1-hue ,1) )' , 'Linewidth', 8 )    
    %     plot( path_nodes( 1 , ii_ ) , path_nodes( 2 , ii_ ) , height_(ii_), 's' , 'Color', squeeze( hsv2rgb( hue ,1,1) )' )        
    plot3( path_nodes( 1 , ii_ ) , path_nodes( 2 , ii_ ) , height_(ii_), 'x' , 'Color', squeeze( hsv2rgb( hue ,1,1) )' , 'Linewidth', 5 )        
    if pause_on 
        pause
    end
end