function [ points_3D__ , points_3D_hom__] = ...
    camera_extrinsics__generate_path_3D_data_sinusoid( ...
    num_datapoints_, num_cycles_, tracks_limit_mins_, tracks_limit_maxes_, feature_heights_)
%{ 
%}

    xy= [ sin( linspace(1,num_cycles_, num_datapoints_)) * (tracks_limit_maxes_(1)-tracks_limit_mins_(1)); 
    linspace(1,num_cycles_, num_datapoints_) *  (tracks_limit_maxes_(2)-tracks_limit_mins_(2))] ;
    xy(1,:) = xy(1,:) + tracks_limit_mins_(1);
    xy(2,:) = xy(2,:) + tracks_limit_mins_(2);

    points_3D_hom__ = zeros(4,num_datapoints_,max(size(feature_heights_)));
    
    for ii_ = 1:max(size(feature_heights_))
        feature_tracks( : , : , ii_) = [ xy; repmat( [feature_heights_(ii_)] , 1 , num_datapoints_ ) ] ;
        points_3D_hom__(:,:,ii_) = geo__euclidean_3D_to_hom(feature_tracks(:,:,ii_));
    end
    
    points_3D__ = feature_tracks;
%     disp(feature_tracks)
%     points_3D_hom__ = geo__euclidean_3D_to_hom(feature_tracks);

end
    
%{
figure; hold on; grid on; axis equal;
plot3_rows(feature_tracks(:,:,1) )
plot3_rows(feature_tracks(:,:,2) )
plot3_rows(feature_tracks(:,:,3) )
plot3_rows(feature_tracks(:,:,1) ,'bx')
plot3_rows(feature_tracks(:,:,2) ,'rs')
plot3_rows(feature_tracks(:,:,3) , 'yd')
%}

