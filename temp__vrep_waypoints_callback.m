function temp__vrep_waypoints_callback( src , event )    
    global stop_sim;

    disp(event.Key);
    if strcmp(event.Key,'leftarrow')
       disp('LEFT ARROW')
    elseif strcmp(event.Key,'rightarrow')
       disp('RIGHT ARROW')
    elseif strcmp(event.Key,'downarrow')       
       disp('downarrow') 
    elseif strcmp(event.Key,'uparrow')
       disp('uparrow')
    elseif strcmpi(event.Key,'q')
        disp('q:  quitting')
        stop_sim = 1 ;
    else
        disp( sprintf('You pressed %s',event.Key) )
   end
end



