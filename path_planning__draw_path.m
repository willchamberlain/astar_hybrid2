%{
RGB triplets:  [1 1 0]
Easier: HSV to RGB  :  [rout,g,b] = hsv2rgb(hin,s,v)
%}

function path_planning__draw_path( total_path_goal_to_start_ )
hold on
path_nodes = flip( total_path_goal_to_start_ , 1);
for ii_ = 1:size(total_path_goal_to_start_,2) 
    hue = ii_ * 1/size(total_path_goal_to_start_,2) ;
    plot( path_nodes( 1 , ii_ ) , path_nodes( 2 , ii_ ) , 's' , 'Color', squeeze( hsv2rgb( 1-hue ,1,1) )' , 'Linewidth', 2 )    
%     plot( path_nodes( 1 , ii_ ) , path_nodes( 2 , ii_ ) , 's' , 'Color', squeeze( hsv2rgb( hue ,1,1) )' )        
    plot( path_nodes( 1 , ii_ ) , path_nodes( 2 , ii_ ) , 'x' , 'Color', squeeze( hsv2rgb( hue ,1,1) )' , 'Linewidth', 1 )        
end