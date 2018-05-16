function points_3D__ = camera_extrinsics__flip_translation_across_path( x_or_y_or_z_, points_3D_ )
    % flip across the x or y or z by negating the values of that axis for all 3D points.
    column_vectors = false ;
    if size(points_3D_,1) == 3 
        column_vectors = true ;
    end
    if max(size(regexp(x_or_y_or_z_,'x'))) > 0
        if column_vectors
            points_3D_(1,:) = points_3D_(1,:).*-1 ;
        else
            points_3D_(:,1) = points_3D_(:,1).*-1 ;
        end
    end
    if max(size(regexp(x_or_y_or_z_,'y'))) > 0
        if column_vectors
            points_3D_(2,:) = points_3D_(2,:).*-1 ;
        else
            points_3D_(:,2) = points_3D_(:,2).*-1 ;
        end
    end
    if max(size(regexp(x_or_y_or_z_,'z'))) > 0
        if column_vectors
            points_3D_(3,:) = points_3D_(3,:).*-1 ;
        else
            points_3D_(:,3) = points_3D_(:,3).*-1 ;
        end
    end
    points_3D__ = points_3D_;
end