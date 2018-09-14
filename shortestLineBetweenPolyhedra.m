%%  addpath( '/mnt/nixbig/ownCloud/project_code/3rd_party/DistBetween2Segment/' )

function [ ...
        min_dist, ...
        FoV1_edge_num, FoV2_edge_num, ...
        FoV1_exit, FoV2_exit] ... 
        = shortestLineBetweenPolyhedra(  FoV_1, FoV_2 )
FoV1_edge_num = -1  ;
FoV2_edge_num = -1  ;
min_dist = 10^16  ; % (max(size(map_image))^2)*2  ;
for ii_ = 1:size(FoV_1,1)
    FoV1_edge_pt1 = FoV_1(ii_,:)  ;
    if ii_ < size(FoV_1,1)
        FoV1_edge_pt2 = FoV_1(ii_+1, :)  ;
    else
        FoV1_edge_pt2 = FoV_1(1,:)  ;
    end
    %plot2_rows( [ edge_pt1 ; edge_pt2 ]' )  ;
    for jj_ = 1:size(FoV_2,1)
        FoV2_edge_pt1 = FoV_2(jj_,:)  ;
        if jj_ < size(FoV_2,1)
            FoV2_edge_pt2 = FoV_2(jj_+1, :)  ;
        else
            FoV2_edge_pt2 = FoV_2(1,:)  ;
        end
        
        [line1_exit__, line2_exit__, line_between_veline2_exit__c_, dist__] = ...
            shortestLineBetweenLineSegments( ...
                FoV1_edge_pt1,FoV1_edge_pt2,...
                FoV2_edge_pt1,FoV2_edge_pt2)  ;
        if dist__ < min_dist
            min_dist = dist__  ;
            FoV1_edge_num = ii_  ;
            FoV2_edge_num = jj_  ;
            FoV1_exit = line1_exit__;
            FoV2_exit = line2_exit__;
        end        
    end    
end