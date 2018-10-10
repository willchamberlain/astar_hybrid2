function send_cmd_vel_planar_safer(x_, tz_,  how_long_s, cmd_vel_publisher_ , cmd_vel_msg ) 
    if how_long_s > 10
        display(sprintf('send_cmd_vel_planar_safer: Limiting command to 10s rather than %fs',how_long_s))
        how_long_s = 10  ;
    end
    
    x=0;
    y=0;
    z=0;
    tx=0;
    ty=0;
    tz=0;        
        
    if abs(x) > 1 || abs(tz)>1
        display(sprintf('send_cmd_vel_planar_safer: Limiting parameters to max 1'))        
        vals_= sign([ x_ ,tz_]) .* min(  abs( [ [x_,tz_] ; ones(1,2) ])   )   
        x=vals_(1)
        tz=vals_(2)
    else
        x=x_;
        tz=tz_;                
    end
    
    cmd_vel_msg.Linear.X =  x ;
    cmd_vel_msg.Linear.Y =  y ;
    cmd_vel_msg.Linear.Z =  z ;
    cmd_vel_msg.Angular.X =  tx ;
    cmd_vel_msg.Angular.Y =  ty ;
    cmd_vel_msg.Angular.Z =  tz ;
    cmd_vel_publisher_.send(cmd_vel_msg)  ;
    pause(how_long_s)
    %{
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
    %}
    cmd_vel_msg.Linear.X =  0 ;
    cmd_vel_msg.Linear.Y =  0 ;
    cmd_vel_msg.Linear.Z =  0 ;
    cmd_vel_msg.Angular.X =  0 ;
    cmd_vel_msg.Angular.Y =  0 ;
    cmd_vel_msg.Angular.Z =  0 ;
    cmd_vel_publisher_.send(cmd_vel_msg)  ;
    %{
    pause(0.15)
    %}
end