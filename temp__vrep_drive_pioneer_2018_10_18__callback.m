function   temp__vrep_drive_pioneer_2018_10_18__callback( src , event, handles )   
    %  see  /mnt/nixbig/ownCloud/project_code/temp__vrep_drive_pioneer_2018_10_16.m
    %    temp__vrep_drive_pioneer_2018_10_16

%     global vrep__simx_opmode_oneshot_wait
%     global vrep__simxGetObjectOrientation
%     global vrep
%     global clientID
%     global cam603_base_cuboid


figure__app_data_handles = guidata(gcf) ;      % handle to 'application data' of current callback object 
        % my_appdata_struct = figure__app_data_handles.my_appdata_struct   %  only one field in figure_child_handles at a time  - so use a struct in that field - figure_child_handles.start_point_pixels = [mousePos(1),mousePos(1)]  ;


vrep = figure__app_data_handles.my_appdata_struct.vrep  ;
clientID = figure__app_data_handles.my_appdata_struct.clientID
cam603_base_cuboid = figure__app_data_handles.my_appdata_struct.cam603_base_cuboid  

    shiftY = 0;
    shiftX = 0;
    angleChangeYaw = 0;
    disp(event.Key)
    if strcmp(event.Key,'asdfdaf3wesfcddf') 
        disp('WTF ?')
    elseif strcmp(event.Key,'numpad7')
       disp('CIRCLE FORWARD LEFT')
       angleChangeYaw = 10 ;
       shiftX = 0.2*sqrt(2) ;
       shiftY = 0.2*sqrt(2) ;       
    elseif strcmp(event.Key,'uparrow')   || strcmp(event.Key,'numpad8')    
       disp('UP ARROW')       
       shiftX = 0.2*2 ;
       shiftY = 0 ;       
    elseif strcmp(event.Key,'numpad9')
       disp('CIRCLE FORWARD RIGHT')
       angleChangeYaw = -10;
       shiftX = 0.2*sqrt(2) ;
       shiftY = -0.2*sqrt(2) ;       
    elseif strcmp(event.Key,'leftarrow') || strcmp(event.Key,'numpad4')
       disp('LEFT ARROW')
       angleChangeYaw = 15 ;       
    elseif strcmp(event.Key,'rightarrow') || strcmp(event.Key,'numpad6')
       disp('RIGHT ARROW')
       angleChangeYaw = -15 ;
    elseif strcmp(event.Key,'numpad1')    
       disp('CIRCLE BACK LEFT')       
       angleChangeYaw = -10 ;
       shiftX = -0.2*sqrt(2) ;
       shiftY = 0.2*sqrt(2) ;       
    elseif strcmp(event.Key,'downarrow')   || strcmp(event.Key,'numpad2')    
       disp('DOWN ARROW')
       shiftX = -0.2 ;
       shiftY = 0 ;       
    elseif strcmp(event.Key,'numpad3')    
       disp('CIRCLE BACK RIGHT')
       angleChangeYaw = 10 ;
       shiftX = -0.2*sqrt(2) ;
       shiftY = -0.2*sqrt(2) ;       
    else
        disp( sprintf('You pressed %s',event.Key) )
    end
    
    
    
    %  Get orientation from VRep one-off.
    [returnCode, orientation_eulerAnglesZYZ] = ...        
        vrep.simxGetObjectOrientation( ...
                clientID, ...
                cam603_base_cuboid, ....
                -1, vrep.simx_opmode_oneshot_wait)  ;
                %cam603_base_cuboid_orientation_as_SE3 = eul2tr(orientation_eulerAnglesZYZ)    %, 'deg')
                
    %  Get position from VRep one-off.
    [returnCode,position] = ...
        vrep.simxGetObjectPosition(clientID,cam603_base_cuboid,-1,vrep.simx_opmode_oneshot_wait)  ;  % relativeToObjectHandle = -1 --> relative to world frame
    cam603_base_cuboid_position = position'  ;
    cam603_base_cuboid_position
    
                    % combine orientation and position into SE3 transform
                    %cam603_base_cuboid_pose = rt2tr(cam603_base_cuboid_orientation_as_SE3,cam603_base_cuboid_position)  ;

display(sprintf('angleChangeYaw=%f shiftX=%f shiftY=%f ',  angleChangeYaw , shiftX , shiftY ));
                    
    %  Apply rotation update.
    rot_ = r2tr(rotz(degtorad(angleChangeYaw))) * eul2tr(orientation_eulerAnglesZYZ)  ;
    eulerAngleZYZ = tr2eul(rot_)   ;  
orientation_eulerAnglesZYZ
eulerAngleZYZ
    [returnCode]= vrep.simxSetObjectOrientation(clientID, cam603_base_cuboid, -1, ...
       eulerAngleZYZ, ...
        vrep.simx_opmode_oneshot_wait)  ;        
    
    
    cam603_base_cuboid_position' 
    [  shiftX shiftY 0 ]
    %  Apply position update.
    posn_now = cam603_base_cuboid_position'  +  [  shiftX shiftY 0 ]  ;
        [returnCode]= vrep.simxSetObjectPosition(clientID, cam603_base_cuboid, -1, posn_now, vrep.simx_opmode_oneshot_wait)  ;
    
end