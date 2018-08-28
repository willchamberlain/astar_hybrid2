%{
Connects to VRep 
Scene 
    /mnt/nixbig/data/project_AA1_backup_pioneer3c/for_vrep/vrep_QUT_S11/
        BoofCV_measurement_model_20180810_02.ttt
Move marker relative to camera
Get image from camera's vision sensor
Save image to file with relative position encoded in the file name.

MIGHT BE BETTER to use the C++ AprilTag code in a class which interacts with VRep directly.
But this will do

%}

%%
addpath('/mnt/nixbig/ownCloud/project_code/')

%%


vrep = VREP();
vrep.loadscene( '/mnt/nixbig/data/project_AA1_backup_pioneer3c/for_vrep/vrep_QUT_S11/BoofCV_measurement_model_20180810_02.ttt' )

camera_obj = vrep.camera( 'Vision_sensor' )
camera_handle = vrep.gethandle( 'Vision_sensor' )
marker_handle = vrep.gethandle( 'BoofCV_3' )

vrep.simstart

%%
for ii_ = -1 : 0.2 : 1
    for jj_ = -1 : 0.2 : 1
        vrep.setpos( marker_handle , [ii_ , jj_ , 2] ,  camera_handle)
        im = camera_obj.grab() ;
        idisp(im)

        camera_pose = vrep.getpose(marker_handle,camera_handle)  ;
        camera_pose_pos = camera_pose(1:3,4)  ;
        camera_pose_quat = Quaternion(camera_pose(1:3,1:3))  ;
        camera_pose_quat.v
        camera_pose_quat.s

        sprintf('ass %10.10f',pi*10^11)
        sprintf('ass %10.10f',pi*10^3)

        filepath = '/mnt/nixbig/ownCloud/project_code/measurement_model_boofcv/measurement_model_boofcv'  ;
        % position xyz
        % orientation Quaternion([S V1 V2 V3])
        filename = ...
        sprintf('x%10.10f_y%10.10f_z%10.10f_s%10.10f_v%10.10f_%10.10f_%10.10f',...
            camera_pose_pos(1), camera_pose_pos(2), camera_pose_pos(3), ...
            camera_pose_quat.s, camera_pose_quat.v(1), camera_pose_quat.v(2), camera_pose_quat.v(3))  ;  
        %  filename = strrep(strrep(filename,'_','__'),'.','_')  ;

        filepathname = strcat( filepath, '/' , filename, '.png' )  
        imwrite(im,filepathname)
        
        drawnow()
        pause(0.1)
    end
end

%%

vrep.simstop



