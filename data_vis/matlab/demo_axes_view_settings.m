


figure_named('test setting view','default'); 

plot3(  [0 0 0 0 ] ,  [ 0 0 0 0 ] ,  [0.0 0.5 1.0 1.5] ,  'rx' )
plot3( 1 , 1 , 0 , 'bo' )

%%
% get the scale to use
    view([45, 45]) 
    [view_azimuth, view_elevation]=view
    xlim([-1, 2]);    ylim([-1, 2]);   zlim([0,2]);
    camera_target = [1;1;0]    
    view(view_angle_3D)
    axes = gca 
    
    axes.CameraPosition
    
    axes.CameraPositionMode='manual'    
    axes.CameraPosition=[20;17;15]
    axes.CameraTargetMode='manual'    
    axes.CameraTarget = [1 1 0]
    axes.CameraUpVector=[ 0 0.01 0.5]
    axes.CameraUpVector=[ 0 0 1]
    axes.CameraUpVectorMode
    axes.CameraViewAngle
    axes.CameraViewAngleMode
    
    axes.DataAspectRatio=[1 1 5]
    axes.DataAspectRatio=[1 1 1]
    axes.DataAspectRatioMode
    
    axes.Box = 'on'
    
    axes.BoxStyle='full'
    axes.BoxStyle='back'
    
    axes.ClippingStyle='rectangle|3dbox'
    
    child_handle = axes.Children    
%     child_handle.Selected='on'
%     child_handle.SelectionHighlight='off'
    child_handle.MarkerEdgeColor
    child_handle.MarkerFaceColor = 'c'
    child_handle.MarkerSize = 10
    child_handle.Marker = 's'
    child_handle.AlignVertexCenters='on'
    child_handle.AlignVertexCenters='off'
    child_handle.HitTest='off'
    child_handle.HitTest='on'
    child_handle.HandleVisibility='off'
    child_handle.HandleVisibility='on'
    
    %axes.Selected='on'  % selection indictors for the whole axes/graph  
    %axes.Selected='off'
    %axes.SelectionHighlight
%%    
for ii_ = 1:2:90
    view([ii_, ii_]) 
    pause(0.1)
end


