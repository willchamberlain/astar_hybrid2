function temp__plot_boofcv_StoT_inVOS_transl(boofcv_StoT_x,boofcv_StoT_y,boofcv_StoT_z, varargin)

    plot3([0 boofcv_StoT_z],[0 -1*boofcv_StoT_x],[0 -1*boofcv_StoT_y],varargin{:}) ; axis equal

end