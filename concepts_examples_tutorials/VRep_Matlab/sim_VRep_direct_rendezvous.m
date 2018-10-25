%%   not using Peter's toolbox

% open connection to VRep - see  http://www.forum.coppeliarobotics.com/viewtopic.php?t=3325
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
%   [rtn] = simxStart(obj,server,port,waitUntilConnected,doNotReconnectOnceDisconnected,timeOutInMs,commThreadCycleInMs)
%   clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,5);  % get a connection = start a connection != start sim
getenv('ROS_MASTER_URI')
clientID=vrep.simxStart('192.168.43.252',11311,true,true,5000,5);  % get a connection = start a connection != start sim
clientID    % is -1 if the connection failed 
% do stuff  -  see  http://www.forum.coppeliarobotics.com/viewtopic.php?t=1445
[retVal,Pioneer_p3dx] = vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_oneshot_wait)  % get a handle to the object by name 
pioneers{1} = Pioneer_p3dx ;
    %   [rtn childObjectHandle] = simxGetObjectChild(obj,clientID,parentObjectHandle,childIndex,operationMode)
    % [rtn childObjectHandle] = vrep.simxGetObjectChild( clientID,Pioneer_p3dx,1,vrep.simx_opmode_oneshot_wait)
    % childObjectHandle.getProperties
    % vrep.simxGetObjects
[retVal,Pioneer_p3dx__base_link] = vrep.simxGetObjectHandle(clientID,'BoofCV_5_Sphere',vrep.simx_opmode_oneshot_wait)  % get a handle to the object by name 

%   function [rtn position] = simxGetObjectPosition(obj,clientID,objectHandle,relativeToObjectHandle,operationMode)
[rtn position] = vrep.simxGetObjectPosition(clientID, Pioneer_p3dx__base_link, -1, vrep.simx_opmode_oneshot_wait)  
Pioneer_p3dx__base_link
    
% Open a streaming pose from VRep
    %          function [rtn position] = simxGetObjectPosition(obj,clientID,objectHandle,relativeToObjectHandle,operationMode)
[retVal,pos] = vrep.simxGetObjectPosition(clientID,cam603_base_cuboid,-1,vrep.simx_opmode_streaming)   % relativeToObjectHandle = -1 --> relative to world frame
% get position from VRep one-off
[retVal,pos] = vrep.simxGetObjectPosition(clientID,cam603_base_cuboid,-1,vrep.simx_opmode_oneshot_wait)   % relativeToObjectHandle = -1 --> relative to world frame

% get orientation from VRep one-off
    %  Matlab remote api functions  -  http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsMatlab.htm
    %  Quaternion(w,x,y,z) - http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctionsMatlab.htm#simxGetObjectQuaternion
    %       Regular/non-Matlab API : VRep defaults to Euler angles http://www.coppeliarobotics.com/helpFiles/en/regularApi/simGetObjectOrientation.htm
    %       NOT in Matlab API - full transformation matrix - http://www.coppeliarobotics.com/helpFiles/en/regularApi/simGetObjectMatrix.htm
    %       Regular/non-Matlab API : full list of coordinate/matrix/transformation related functions -  http://www.coppeliarobotics.com/helpFiles/en/apiFunctionListCategory.htm#coordinatesAndTransformations
    %       LATER VERSION OF VREP  [number returnCode,array quat]=vrep.simxGetObjectQuaternion(clientID,cam603_base_cuboid,-1,simx_opmode_oneshot_wait)
    % 
    % [number returnCode,array eulerAngles]
    % [ status code, ZYZ Euler angles ]
    %       =simxGetObjectOrientation(  number clientID,  number objectHandle,  number relativeToObjectHandle,  number operationMode)
    % 

%     figure_named('eyeball check of pose of cam603_base_cuboid','default')
%     time_as_vertical_offset = [0;0;0.0]
%     while true
        % get position from VRep one-off
        [returnCode, orientation_eulerAnglesZYZ] = vrep.simxGetObjectOrientation(clientID, cam603_base_cuboid, -1, vrep.simx_opmode_oneshot_wait)
        cam603_base_cuboid_orientation_as_SE3 = eul2tr(orientation_eulerAnglesZYZ) %, 'deg')
        % get position from VRep one-off
        [retVal,position] = vrep.simxGetObjectPosition(clientID,cam603_base_cuboid,-1,vrep.simx_opmode_oneshot_wait)   % relativeToObjectHandle = -1 --> relative to world frame
        cam603_base_cuboid_position = position'
        cam603_base_cuboid_pose = rt2tr(cam603_base_cuboid_orientation_as_SE3,cam603_base_cuboid_position)

        draw_axes_direct(tr2r(cam603_base_cuboid_pose),tr2t(cam603_base_cuboid_pose)+time_as_vertical_offset,'',1)
        pause(0.5)
%         time_as_vertical_offset = time_as_vertical_offset+[0;0;0.05];
%     end


% Demonstrate moving object through a series of poses
% 	[number returnCode]
%       =simxSetObjectOrientation(number clientID,number objectHandle,number relativeToObjectHandle,  array eulerAngles,number operationMode)
%   http://www.coppeliarobotics.com/helpFiles/en/remoteApiConstants.htm#functionErrorCodes
ii_ = 0
for y_ = 90:-5:0
    ii_ = ii_ + 1;
    yaw = y_
    rot_(:,:,ii_) = rotz(degtorad(yaw))
end
size(rot_)  %  3  3  19
eulerAngleZYZ = tr2eul(rot_)   
size(eulerAngleZYZ)   % 19  3
for ii_ = 1:size(eulerAngleZYZ,1)
    [returnCode]= vrep.simxSetObjectOrientation(clientID, cam603_base_cuboid, -1, squeeze(eulerAngleZYZ(ii_,:,:)), vrep.simx_opmode_oneshot_wait)
    pause(0.2)
end
%         function rtn = simxSetObjectPosition(obj,clientID,handle,rel_pos,position,option)
for ii_ = 1:size(eulerAngleZYZ,1)
    posn_now = cam603_base_cuboid_position'+ii_*[ 0 0 0.2 ]
    [returnCode]= vrep.simxSetObjectPosition(clientID, cam603_base_cuboid, -1, posn_now, vrep.simx_opmode_oneshot_wait)
    pause(0.2)
end

% close connection
vrep.simxFinish(clientID); % close the line if still open = finish connection
vrep.delete(); % call the destructor!
