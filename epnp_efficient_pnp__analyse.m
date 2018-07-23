%{
Doco for 
/mnt/nixbig/downloads/MachineVisionToolkit/vision-3.4/rvctools/contrib/EPnP/EPnP/efficient_pnp.m 
    Copyright (C) <2007>  <Francesc Moreno-Noguer, Vincent Lepetit, Pascal Fua>
/mnt/nixbig/downloads/MachineVisionToolkit/vision-3.4/rvctools/contrib/EPnP/EPnP/efficient_pnp_will.m

Writing this in Matlab because I like the syntax-colouring at the moment.


%

%       A: intrincic camera parameters
%       R: Rotation of the camera system wrt world reference
%       T: Translation of the camera system wrt world reference
%       Xc: Position of the points in the camera reference
%}

 [  R  ,T  ,... %       R: Rotation, T: Translation,  of the camera system wrt world reference
     Xc  ,...  %       Xc: Position of the points in the camera reference  e.g. pixels coordinates, normalised image coordinates, etc
     best_solution  , ...
     solution_set  , ...
     Alph  ]  ... 
     =  efficient_pnp_will( ... 
     x3d_h , ... %       x3d_h: homogeneous coordinates of the points in world reference : assumes scale factor is 1
     x2d_h , ... %       x2d_h: homogeneous position of the points in the image plane : assumes scale factor is 1
     cam_intrinsics   ) %       cam_intrinsics/'A': intrincic camera parameters
 
Xw  =  x3d_h(:,1:3);  %  convert homogeneous straight to 3-vec 
U   =  x2d_h(:,1:2); % image coordinates - convert straight to 2-vec 

THRESHOLD_REPROJECTION_ERROR  =  20  ;  %  error in degrees of the basis formed 
%  by the control points  : over this threshold, try again "using a larger number of vectors in the kernel". 


control points in world coordinate system 

alphas as 3D datapoints expressed as linear combinations of the control points : datapoints in control-point space

M =  compute_M_ver2(U, Alph  ,A) = (  2D points , alphas , camera intrinsics ) 
    %------------------------------
    compute_M_ver2(U, Alph  ,A) = (  2D points , alphas , camera intrinsics )    
        M is  (2*num_data)x12 =  2*num_3D_points  by  12 columns =  2*alphas  by  12 columns
        for each alpha
            submatrix = 2x12 combination of alphas and image coordinates
            stack submatrices into M
        return M = two rows per datapoint of the 2D datapoints image coordinate in terms of the alphas for that point 
            = (( "from the use in the Solve assuming dim..."  sections ))  one column per dimension of kernel 
            
    %------------------------------

"Kernel of M"  / "Kernel with M" / ?
kernel_noise( M , 4 )
note comment :  "%in matlab we have directly the funcion km=null(M);"
    -->  is calculating the null space of M ? 
    
    %------------------------------
    kernel_noise( M , 4 )  =  kernel_noise(M,kernel_dimension)
        
        [  full column matrix of eigenvectors  ,  diagonal of eignevalues   ] =  eig( M_transpose * M )  =  eig(A_)
            % such that A_*matrix_of_eigenvectors =  matrix_of_eigenvectors * diagonal_of_eignevalues
            % therefore this is the eigenvectors and eigenvalues of M-ish ??
    
        return full column matrix of eigenvectors ( : ,  kernel_dimension: -1: 1 )   % e.g. 3,2,1
    %------------------------------
    
    Note: different algorithms for each kernel size
    solve for kernel size 1  -->  err(1) , sol(1)
    solve for kernel size 2  -->  err(2) , sol(2)
            D = compute_constraint_distance_2param_6eq_3unk
                = function of the 2 eigenvectors
    if err(1) > thresh && err(2) > thresh 
        solve for kernel size 3  -->  err(3) , sol(3)
            6x6 = D = compute_constraint_distance_3param_6eq_6unk
                = function of the 3 eigenvectors
    if err(1) > thresh  &&  err(2) > thresh  &&  err(3) > thresh  
        solve for kernel size 4  -->  err(4) , sol(4)
            D = compute_constraint_distance_orthog_4param_9eq_10unk
                = function of the 4 eigenvectors

    pick best solution as min(err)
    return best solution: in my version return the other data as well 
    

