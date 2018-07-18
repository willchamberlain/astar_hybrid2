function test_params__icanny(im_ )
%  (1) Test changing the parameters of icanny applied to an image - e.g. cleaning up floorplans from gmapping.
%  (2) Exercise in using callback functions, handling keypresses.
%  (3) Exercise in exercising functions, algorithms, or pipelines through a UI.
%  Note: (3) might be achievable through Matlab Live Scripts in more recent Matlab versions.
%
%  This function body sets up the figure and its callback function, sets the algorithm's default parameters, and calls the algorithm invocation function.
%    Parameters for the functions, algorithms, and pipelines are held in a structure in named attributes.
%  The callback function interprets the user inputs, alters the algorithm parameters, and calls the algorithm invocation function.
%  The algorithm invocation function invokes the functions, algorithms, and pipelines with the current parameter values (and variations), and displays the
%  output along with the parameter values.

    global test_params__icanny__variables; 
    test_params__icanny__variables.im = im_ ;
    test_params__icanny__variables.factor_scalar = 1.0  ;
    test_params__icanny__variables.factor_smooth = 1.0  ;

    figure('KeyPressFcn',@a_callback)  ;
    
    setup(test_params__icanny__variables.im,test_params__icanny__variables.factor_scalar, test_params__icanny__variables.factor_smooth) ;

end

function  a_callback(src_, eventdata_)  
    global test_params__icanny__variables; 
    display(eventdata_.Key)
    switch eventdata_.Key
        case 'hyphen'
            test_params__icanny__variables.factor_scalar = test_params__icanny__variables.factor_scalar/1.1  ;
        case 'equal'
            test_params__icanny__variables.factor_scalar = test_params__icanny__variables.factor_scalar*1.1  ;
        case 'leftbracket'
            test_params__icanny__variables.factor_smooth = test_params__icanny__variables.factor_smooth/1.1  ;
        case 'rightbracket'
            test_params__icanny__variables.factor_smooth = test_params__icanny__variables.factor_smooth*1.1  ;
    end
    setup(test_params__icanny__variables.im, test_params__icanny__variables.factor_scalar, test_params__icanny__variables.factor_smooth)
end

function setup(im_, factor_scalar_, factor_smooth_) 
    canny_factor_std_dev_smooth = factor_smooth_ ;
    canny_factor_im_threshold = factor_scalar_  ;
    h_axis_icanny_test(1) = subplot( 2,2,1) ; imshow( icanny( 100*(im_>50*canny_factor_im_threshold), 'sd',canny_factor_std_dev_smooth)  )
    set(h_axis_icanny_test(1), 'Position' , [ 0.01             0.5+0.01 0.5-0.01*1.5    0.5-0.01*1.5 ] )
    text(  10 , 10 ,  sprintf('imthresh: %5.2f, sd: %5.2f + erode',50*canny_factor_im_threshold,canny_factor_std_dev_smooth) , 'Color' , 'g')
    %     
    h_axis_icanny_test(2) = subplot( 2,2,2) ; imshow( icanny( 100*(im_>100*canny_factor_im_threshold), 'sd',canny_factor_std_dev_smooth))
    set(h_axis_icanny_test(2), 'Position' , [ 0.5+0.01/2  0.5+0.01  0.5-0.01*1.5    0.5-0.01*1.5 ] )
    text(  10 , 10 ,  sprintf('imthresh: %5.2f, sd: %5.2f',100*canny_factor_im_threshold,canny_factor_std_dev_smooth) , 'Color' , 'g')
    %
    h_axis_icanny_test(3) = subplot( 2,2,3) ; imshow( icanny( 100*(im_>150*canny_factor_im_threshold), 'sd',canny_factor_std_dev_smooth))
    set(h_axis_icanny_test(3), 'Position' , [ 0.01              0.01         0.5-0.01*1.5   0.5-0.01*1.5 ] )
    text(  10 , 10 ,  sprintf('imthresh: %5.2f, sd: %5.2f',150*canny_factor_im_threshold,canny_factor_std_dev_smooth) , 'Color' , 'g')
    %
    h_axis_icanny_test(4) = subplot( 2,2,4) ; imshow( icanny( 100*(im_>200*canny_factor_im_threshold), 'sd',canny_factor_std_dev_smooth))
    set(h_axis_icanny_test(4), 'Position' , [ 0.5+0.01/2    0.01        0.5-0.01*1.5    0.5-0.01*1.5 ] )
    text(  10 , 10 ,  sprintf('imthresh: %5.2f, sd: %5.2f',200*canny_factor_im_threshold,canny_factor_std_dev_smooth) , 'Color' , 'g')
end



