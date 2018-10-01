function [x__,y__,z__] = ...
    temp__boofcv_TtoS_to_VOS_StoT_transl(...
    boofcv_StoT_x,boofcv_StoT_y,boofcv_StoT_z)
        x__ = boofcv_StoT_z  ;
        y__ = -1*boofcv_StoT_x  ;
        z__ = -1*boofcv_StoT_y  ;
end