% https://www.scratchapixel.com/lessons/3d-basic-rendering/computing-pixel-coordinates-of-3d-point/mathematics-computing-2d-coordinates-of-3d-points
P_world = [ 1 0 0 1 ]'
T_world_to_local = [ ...
    1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1;
    ];
P_camera = T_world_to_local * P_world  
camera_est = T_world_to_local; P_camera = camera_est * P_world  
P_world = [ 1 2 3 1 ]'
P_camera = T_world_to_local * P_world  
camera_est = T_world_to_local; P_camera = camera_est * P_world  
T_world_to_local = [ ...
    1 0 0 1;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1;
    ];
P_camera = T_world_to_local * P_world  
camera_est = T_world_to_local; P_camera = camera_est * P_world  
T_local_to_world = vertcat(  horzcat(T_world_to_local(1:3,1:3)' , T_world_to_local(1:3,1:3)'*T_world_to_local(1:3,1)*-1 ) , [ 0 0 0 1 ] )
P_camera_world = T_local_to_world * P_camera

P_world = [ 10 2 3 1 ]'
P_camera = T_world_to_local * P_world  
camera_est = T_world_to_local; P_camera = camera_est * P_world  

P_cam_origin_in_local = [ 0 0 0 1 ]'
P_camera_origin_in_world = T_local_to_world * P_cam_origin_in_local
P_cam_origin_in_local = T_world_to_local * P_camera_origin_in_world 

yaw_world_to_local = (2*pi)/8 
R_yaw_world_to_local = [...    
    cos(yaw_world_to_local ) -sin(yaw_world_to_local )  0;
    sin(yaw_world_to_local ) cos(yaw_world_to_local )   0;
    0   0   1;
    ]
T_world_to_local = [ ...
    R_yaw_world_to_local , [ 1 0 0 ]'
    ] ;
T_world_to_local = [ T_world_to_local ; [ 0 0 0 1 ] ]
T_local_to_world = vertcat(  horzcat(T_world_to_local(1:3,1:3)' , T_world_to_local(1:3,1:3)'*T_world_to_local(1:3,1)*-1 ) , [ 0 0 0 1 ] )

P_object_in_camera = [ 1 0 0 1 ]'
P_object_in_world = T_local_to_world * P_object_in_camera

%% 
% Camera matrix is set up for a RDF coordinate system.
camera_K = [...
   322.9596901156589 , 000.0000000000000 , 176.8267919600727 ; %,    //  f_x ,   0 , c_x
                000.0000000000000 , 323.8523693059909 , 146.7681514313797 ; %,    //    0 , f_y , c_y
                  0.0 ,               0.0 ,               1.0 ];              
proj_ = horzcat( camera_K , [ 0 ; 0 ; 0  ]) * [ 0 0 10 1 ]'

%%   % wrong somewhere in this section 
% 90 pitch
R_pitch_FLU_to_RDF = [...    % wrong somewhere in this section 
    cos(pi/2)       0     sin(pi/2) ;
    0                   1       0 ;
    -sin(pi/2)       0       cos(pi/2) ;
    ] ;
% then 
% -90 yaw
R_yaw_FLU_to_RDF = [...    
    cos(-pi/2)   -sin(-pi/2)    0;
    sin(-pi/2)      cos(-pi/2)   0;
    0                   0                   1;
    ] ;
R_yaw_FLU_to_RDF * R_pitch_FLU_to_RDF  *  [ 1 0 0 ]'     ; 
R_FLU_to_RDF = R_yaw_FLU_to_RDF * R_pitch_FLU_to_RDF    
T_FLU_to_RDF = r2t(R_FLU_to_RDF)
round( T_FLU_to_RDF )
T_FLU_to_RDF *  [ 1 0 0 1 ]' 
T_FLU_to_RDF = [...
    0 -1  0  0 
    0  0  1  0
    -1  0  0  0
    0  0  0  1   
    ]
 T_FLU_to_RDF = [ ...
     0    -1     0     0
     0     0    -1     0
     1     0     0     0
     0     0     0     1   ];


%%


