function centralCamera_ = camera_extrinsics__flip_camera_across_path( x_or_y_or_z_, centralCamera_ )

    centralCamera_position = centralCamera_.T(1:3,4)  ; 
    flipped_translation = camera_extrinsics__flip_translation_across_path(x_or_y_or_z_,centralCamera_position )  ; 
        
    translation_only = eye(4)  ;    
    if max(size(regexp(x_or_y_or_z_,'x'))) > 0    
        translation_only(1,4) = flipped_translation(1)*2  ;
    end
    if max(size(regexp(x_or_y_or_z_,'y'))) > 0
        translation_only(2,4) = flipped_translation(2)*2  ;
    end
    if max(size(regexp(x_or_y_or_z_,'z'))) > 0
        translation_only(3,4) = flipped_translation(3)*2  ;
    end           
    
    display('TODO - this assumes flip across zero axes, not across the path axis at the point normal to the cameras current position.')
    
    centralCamera_.T = centralCamera_.T+(translation_only);
    
end
