function measurement_model_for_BoofCVSquareFiducial
%{
Determine a measurement model for BoofCV Square Fiducial markers.

Why : show how the image formation and algorithm contributes to the uncertainty of pose estimation.

Hypotheses / results / analyses : 
    NOTE:  just need illustrative values - will not get a paper out of this (except with regard to handling different measurement models).

    Tag pose estimate accuracy and detection reliability are both inverse non-linear to relative orientation of tag and image plane
        Pose accuracy and detection reliability over a range of marker angles to the optical axis - marker on the optical axis, rotate it in pitch, and in yaw, and in roll.
    
   Tag pose estimate accuracy and detection reliability are both inverse to distance from camera and to offset from camera optical axis
        Pose accuracy and detection reliability over a range of translations from the camera - move the marker to grid points in space within the FoV.

    Tag pose estimate accuracy and detection reliability are both inverse to relative pose to camera image plane
        Pose accuracy and detection reliability over a range of relative poses from the camera - move the marker to grid points in space within the FoV, and correct the orientation relative to the camera-marker ray..

    Detection reliability is inverse to pixel noise in the image
        Move the marker to grid points in space within the FoV, correct to relative orientation to ray to camera centre, 
            apply varying magnitude of intensity of pixel noise
            apply varying ratio pixel noise            
    


System setup
    Tags and image formation : VRep
    Tag detection : 

Experimental process
    1) pose a tag in VRep
    2) take an image
    3) apply camera realism 
        3.1) distortion
        3.2) pixel noise
    4) run detection 

    Note that in the real world not every frame of the same pose will result in a detection: 
    should be able to determine why with this.

%}
end