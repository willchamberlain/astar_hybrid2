function  [detected__, marker_2D_uv__,pose_3D__] = detect_marker_and_request_posn( vrep )
    detected__ = false ;
    marker_2D_uv__ = [-1,-1]  ;
    pose_3D__ = [ 0 0 0 ]  ;
    cam603sensor0 = vrep.camera('cam603sensor0') ;
    im =  cam603sensor0.grab  ;
    [label, num_sets] = ilabel(im2bw(im))  ;
    if num_sets > 1
        detected__ = true;
        marker_pixels = label==2 ;
        blob =  imoments(marker_pixels)  ;
        marker_2D_uv__ = [blob.uc,blob.vc]  ;
        marker_handle = vrep.gethandle('BoofCV_5_Sphere')  ;
        pose_3D__ = double(vrep.getpos(marker_handle))  ;
    end
end