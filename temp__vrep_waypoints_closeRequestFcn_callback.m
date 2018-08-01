function temp__vrep_waypoints_closeRequestFcn_callback( obj, event, handles )    
    global stop_sim;
    stop_sim = 1 ;
    delete(gcf)
end
