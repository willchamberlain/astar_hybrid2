keep = xyz_marker_RDF(3,:)~=0

xyz_marker_RDF(3,1:10)

fitdist(xyz_marker_RDF(1,keep)','normal')
fitdist(xyz_marker_RDF(2,keep)','normal')
fitdist(xyz_marker_RDF(3,keep)','normal')
figure;plot(xyz_marker_RDF(1,:))
figure;plot(xyz_marker_RDF(1,keep))

keepers_qut=rotation_quat_marker_RDF(keep)
theta_degs=radtodeg(tr2rpy(keepers_qut.T))

plot(abs(theta_degs(:,1)))
pd=fitdist(theta_degs(:,1),'normal')
pd.mean
pd.std
theta_degs_minus_outliers= abs(theta_degs( (abs(theta_degs(:,1))<= pd.mean+pd.std & abs(theta_degs(:,1))>= pd.mean-pd.std), :))
plot(theta_degs_minus_outliers(:,1))
fitdist(theta_degs_minus_outliers(:,1),'normal')


keepers_quat_002 = keepers_qut
xyz_camera_FLU_002 = xyz_camera_FLU
rotation_quat_camera_FLU_002= rotation_quat_camera_FLU
rotation_quat_marker_RDF_002 = rotation_quat_marker_RDF
xyz_marker_RDF_002 = xyz_marker_RDF
clear xyz_camera_FLU
clear rotation_quat_camera_FLU
clear rotation_quat_marker_RDF
clear xyz_marker_RDF
clear keepers_qut

% xyz_camera_FLU=xyz_camera_FLU(:,2:150)
% rotation_quat_camera_FLU=rotation_quat_camera_FLU(:,2:150)
% rotation_quat_marker_RDF=rotation_quat_marker_RDF(:,2:150)
% xyz_marker_RDF=xyz_marker_RDF(:,2:150)
% keepers_qut=keepers_qut(:,2:150)


theta_degs_001=radtodeg(tr2rpy(keepers_quat_001.T))
figure('Name','theta_degs_001(:,1)');plot(abs(theta_degs_001(:,1)))
figure('Name','theta_degs_001(:,2)'); plot(theta_degs_001(:,2))
fitdist(abs(theta_degs_001(:,1)),'normal')   
