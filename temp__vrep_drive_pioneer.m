%%
%{ 
Scene : /mnt/nixbig/ownCloud/project_code/vrep_QUT_S11/S11_matlab_20180801_002_one_Pioneer.ttt
%}
pioneer_handle = vrep.gethandle('Pioneer_p3dx')
class(pioneer_handle)
pioneer_pose = vrep.getpose(pioneer_handle)
pioneer_object = vrep.object('Pioneer_p3dx')
class(pioneer_object)

pioneer_z_ = pioneer_pose(3,4)
pioneer2_ = pioneer_object
pioneer_pose__orig_local = pioneer_object.getpose

way_pts = [
    300.0, 500.0 ; 
    100.0, 500.0 ; 
    120.0, 400.0 ; 
    90.0, 80.0 ; 
    100, 10 ; 
    500.0, 500.0]  ;
class(way_pts)
way_pts = [
    200.0, 200.0 ; 
    100.0, 150.0 ; 
    120.0, 400.0 ; 
    90.0, 80.0 ; 
    100, 10 ; 
    500.0, 500.0]  ;

%{
way_pts = [  ...
    pioneer_pose__orig_local(1:2,4)'.*100 + [ 0 50 ] 
    pioneer_pose__orig_local(1:2,4)'.*100 + [ 100 200 ]
    pioneer_pose__orig_local(1:2,4)'.*100 + [ 150 220 ]
    pioneer_pose__orig_local(1:2,4)'.*100 + [ 200 100 ] 
    pioneer_pose__orig_local(1:2,4)'.*100 + [ 250 120 ]
    pioneer_pose__orig_local(1:2,4)'.*100 ]  ;
way_pts=double(round(way_pts))  ;
class(way_pts)
%}    

pioneer_object.setpose( pioneer_pose__orig) ; pioneer_object.setorient(eye(3))

pp = temp__pure_pursuit_vrep_waypoints( ...
    pioneer_z_ , pioneer2_ , ...
    way_pts , vrep ,  pioneer_object )

%{  
    RESET THE POSE
    pioneer_object.setpose( pioneer_pose__orig) ; pioneer_object.setorient(eye(3))
    pioneer_object.setpose( pioneer_pose__orig_local) ; pioneer_object.setorient(eye(3))
%}

