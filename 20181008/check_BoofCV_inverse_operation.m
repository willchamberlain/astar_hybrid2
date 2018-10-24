%%  Check whether BoofCV is doing the inverse correctly:  is for the orientation

% HI :  type=[target-to-camera] feature_id=[970]  
transl=[0.5623277478935382,0.26833144376052354,1.9403640348398326]'
rot=[0.763257392251248, 0.005983407973038192, -0.646066832458417; 0.016387411346480352, -0.9998147004767358, 0.010100369299007188; -0.6458866819521712, -0.01829654447274848, -0.7632140135880443 ] 
target_to_camera_tr = rt2tr(rot,transl)



% HI :  type=[camera-to-target] feature_id=[970]  
transl=[0.8221015401984686,0.23812474397285655,1.8510347484998735]' 
rot=[0.763679060413701, 0.01687782215636526, -0.6453754192754046; 0.005981165273965579, -0.999800276648129, -0.019069149857563892; -0.6455683684534208, 0.010702613399671435, -0.7636274849157837 ] 
camera_to_target_tr = rt2tr(rot,transl)

% very close: don't expect exact because may be sequential observations of the same tag.
tr_invert(target_to_camera_tr)  - camera_to_target_tr

target_to_camera_inverse_tr = tr_invert(target_to_camera_tr)
target_to_camera_inverse_tr - camera_to_target_tr

% Check orientation properly
t2c_quat = Quaternion(target_to_camera_tr(1:3,1:3))
c2t_quat = Quaternion(camera_to_target_tr(1:3,1:3))
t2c_inverse_quat = Quaternion(target_to_camera_inverse_tr(1:3,1:3))

c2t_quat.inner(c2t_quat)
c2t_quat.inner(t2c_inverse_quat)
t2c_inverse_quat.inner(c2t_quat)
c2t_quat.inner(t2c_quat)
t2c_inverse_quat.inner(t2c_quat)
