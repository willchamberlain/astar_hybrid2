function points_3D_with_latency__ = camera_extrinsics__latency_between_track_points_as_ratio(points_3D_ , tracks_heights_ , vel_ratio_)
% gets 4 differences : no final speed estimate, so re-use the last one calculated
diffs_ =  ...
    points_3D_(1:3, (1+size(tracks_heights_,2)):size(tracks_heights_,2):end) ...
    - points_3D_(1:3, 1:size(tracks_heights_,2):end-size(tracks_heights_,2))  ;
diffs_(:,end+1)=diffs_(:,end)   ;
points_3D_with_latency__ = zeros(size(points_3D_)) ;
for ii_ = 1 : size(tracks_heights_,2)
    points_3D_with_latency__(: , ii_:size(tracks_heights_,2):end )   ...
        = points_3D_(: , ii_:size(tracks_heights_,2):end ) + diffs_*vel_ratio_;
end


