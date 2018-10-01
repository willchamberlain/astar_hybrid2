function temp__plot_boofcv_transl_TtoS(boofcv_TtoS_StoT_x,boofcv_TtoS_StoT_y,boofcv_TtoS_StoT_z, varargin)
    plot3([0 boofcv_TtoS_StoT_z],[0 -1*boofcv_TtoS_StoT_x],[0 -1*boofcv_TtoS_StoT_y],varargin{:}) ; axis equal
end