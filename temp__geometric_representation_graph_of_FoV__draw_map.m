function figure_handle__ = temp__geometric_representation_graph_of_FoV__draw_map( ...
    corridor, cam_FoVs , cam_s  )

    figure_handle__ = figure; axis equal; grid on; hold on ; axis equal; grid on; 
    patch(corridor(:,1),corridor(:,2),'blue')  ;
    
    for ii_ = 1 : size( cam_FoVs,3 ) 
        cam_FoV = squeeze(cam_FoVs(:,:,ii_))  ;
        %
        patch(cam_FoV(:,1),cam_FoV(:,2),'green')  ;
        text(cam_FoV(:,1),cam_FoV(:,2),'2')  ; 
    end

    for ii_ = 1 : size( cam_s , 3)
        cam_x = squeeze(cam_s(:,:,ii_))  ;
        %
        draw_cam_2d(cam_x(1),cam_x(2),0.3,'black')
    end    
    
end