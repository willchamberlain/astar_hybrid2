function temp__plot_boofcv_StoT_inVOS_rot(boofcv_StoT_qw, boofcv_StoT_qx,boofcv_StoT_qy,boofcv_StoT_qz, boofcv_StoT_x,boofcv_StoT_y,boofcv_StoT_z)
    %     quat = Quaternion(boofcv_StoT_qw, [boofcv_StoT_x,boofcv_StoT_y,boofcv_StoT_z] ) ;
    quat = Quaternion(boofcv_StoT_qw, [boofcv_StoT_qz,-1*boofcv_StoT_qy,-1*boofcv_StoT_qz] )  ;
    posn = [  boofcv_StoT_z  ;  -1*boofcv_StoT_x  ;  -1*boofcv_StoT_y]  ;
    draw_axes_direct(quat.R,  posn, '', 0.5)  ;
    %       plot3([0 boofcv_StoT_z],[0 -1*boofcv_StoT_x],[0 -1*boofcv_StoT_y],varargin{:}) ; 
    axis equal
end