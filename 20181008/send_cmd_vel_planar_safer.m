function send_cmd_vel_planar_safer(x, tz,  how_long_s, cmd_vel_publisher_ , cmd_vel_msg ) 
    if how_long_s > 10
        display(sprintf('send_cmd_vel_planar_safer: Limiting command to 10s rather than %fs',how_long_s))
        how_long_s = 10  ;
    end
    if abs(x) > 1 || abs(tz)>1
        display(sprintf('send_cmd_vel_planar_safer: Limiting parameters to max 1'))        
        vals_= sign([ x ,tz]) .* min(  abs( [ [x,tz] ; ones(1,2) ])   )   ;
        x=vals_(1);
        y=0;
        z=0;
        tx=0;
        ty=0;
        tz=vals_(6);        
    end
    
    cmd_vel_msg.Linear.X =  x ;
    cmd_vel_msg.Linear.Y =  y ;
    cmd_vel_msg.Linear.Z =  z ;
    cmd_vel_msg.Angular.X =  tx ;
    cmd_vel_msg.Angular.Y =  ty ;
    cmd_vel_msg.Angular.Z =  tz ;
    cmd_vel_publisher_.send(cmd_vel_msg)  ;
    pause(how_long_s)
    cmd_vel_msg.Linear.X =  x*0.5 ;
    cmd_vel_msg.Linear.Y =  y*0.5 ;
    cmd_vel_msg.Linear.Z =  z*0.5 ;
    cmd_vel_msg.Angular.X =  tx*0.5 ;
    cmd_vel_msg.Angular.Y =  ty*0.5 ;
    cmd_vel_msg.Angular.Z =  tz*0.5 ;
    cmd_vel_publisher_.send(cmd_vel_msg)  ;
    pause(0.15)
    cmd_vel_msg.Linear.X =  x*0.25 ;
    cmd_vel_msg.Linear.Y =  y*0.25 ;
    cmd_vel_msg.Linear.Z =  z*0.25 ;
    cmd_vel_msg.Angular.X =  tx*0.25 ;
    cmd_vel_msg.Angular.Y =  ty*0.25 ;
    cmd_vel_msg.Angular.Z =  tz*0.25 ;
    cmd_vel_publisher_.send(cmd_vel_msg)  ;
    pause(0.15)
    cmd_vel_msg.Linear.X =  0 ;
    cmd_vel_msg.Linear.Y =  0 ;
    cmd_vel_msg.Linear.Z =  0 ;
    cmd_vel_msg.Angular.X =  0 ;
    cmd_vel_msg.Angular.Y =  0 ;
    cmd_vel_msg.Angular.Z =  0 ;
    cmd_vel_publisher_.send(cmd_vel_msg)  ;
    pause(0.15)
end