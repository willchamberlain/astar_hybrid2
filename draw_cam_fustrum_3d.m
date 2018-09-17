function draw_cam_fustrum_3d( fustrum_corners, camera_optical_centre,  colour_spec_)
    for ii_ = 1:size(fustrum_corners,2)-1
        plot3_rows( [ fustrum_corners(:,ii_) fustrum_corners(:,ii_+1) ] , colour_spec_)  ;
    end
    plot3_rows( [ fustrum_corners(:,size(fustrum_corners,2)) fustrum_corners(:,1) ] , colour_spec_ )  ;
    for ii_ = 1:size(fustrum_corners,2)
        plot3_rows( [ fustrum_corners(:,ii_) camera_optical_centre ] ,colour_spec_ )  ;
    end
end