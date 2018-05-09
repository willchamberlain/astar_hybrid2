%{
--- 2018_04_06 Sunday ---
-------------------------------------
http://www.opengl-tutorial.org/intermediate-tutorials/tutorial-17-quaternions
    RotationBetweenVectors
    LookAt
    RotateTowards
%}

function quaternion__ = rotation_between_vectors( vec3_start_ , vec3_dest_ ) 
    vec3_start_normalised = normalise_vector(vec3_start_)  ;
    vec3_dest_normalised = normalise_vector(vec3_dest_)  ;
    
    cosTheta = dot( vec3_start_normalised , vec3_dest_normalised )  ;
    min_angular_dsicrimination_threshold = 0.0000001  ;
		% special case when vectors in opposite directions:
		% there is no "ideal" rotation axis
		% So guess one; any rotation axis will do as long as it's perpendicular to  vec3_start_
    if cosTheta <   -1  +  min_angular_dsicrimination_threshold 
        %         display( 'cosTheta ~~ <   -1' )  ;
        FLU_forward_unit_vector = [ 1 0 0 ]   ;
        FLU_left_unit_vector = [ 0 1 0 ]   ;
        if (1-10^-15)  < dot( FLU_forward_unit_vector , vec3_start_normalised )  ||   (1-10^-15)  < dot( FLU_forward_unit_vector , vec3_dest_normalised ) 
            %             display( 'cosTheta ~~ <   -1  &&  (1-10^-15)  < dot( FLU_forward_unit_vector , vec3_start_normalised )  ||   (1-10^-15)  < dot( FLU_forward_unit_vector , vec3_dest_normalised )  ' )  ;
            rotationAxis = cross( FLU_left_unit_vector , vec3_start_normalised )   ;
        else
            %             display( 'cosTheta ~~ <   -1  && not forward vector   ' )  ;
            rotationAxis = cross( FLU_forward_unit_vector , vec3_start_normalised )   ;
        end
        rotationAxis = normalise_vector( rotationAxis )   ;
        quaternion__ = Quaternion(degtorad(180), rotationAxis)   ;   %  Q = Quaternion(TH, V) is a unit-quaternion corresponding to rotation of TH about the vector V  
        return  ; 
    end
    rotationAxis = cross( vec3_start_normalised , vec3_dest_normalised )   ;    
    s = sqrt( (1+cosTheta)*2 )  ;
	s_inv = 1 / s  ;
    quaternion__ = Quaternion(  ...  %  Q = Quaternion([S V1 V2 V3]) is a quaternion formed by specifying directly its 4 elements
        [  s * 0.5  , ...
		  rotationAxis(1) * s_inv  ,  rotationAxis(2) * s_inv  ,  rotationAxis(3) * s_inv  ]  )   ;    
end