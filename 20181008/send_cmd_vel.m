function send_cmd_vel(x,y,z,  tx,ty,tz,  how_long_s, cmd_vel_publisher_ , cmd_vel_msg ) 
    if how_long_s > 10
        display(sprintf('Long command for %f seconds',how_long_s))
    end
    if abs(x) > 1 || abs(y)> 1 || abs(z) > 1 || abs(tx)>1 || abs(ty)>1 || abs(tz)>1
        display(sprintf('limiting parameters to max 1'))
        
        vals_= sign([ x,y,z,  tx,ty,tz]) .* min(  abs( [ [x,y,z,  tx,ty,tz] ; ones(1,6) ])   )   ;
        x=vals_(1);
        y=vals_(2);
        z=vals_(3);
        tx=vals_(4);
        ty=vals_(5);
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