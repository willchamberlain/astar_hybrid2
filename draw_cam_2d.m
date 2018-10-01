function draw_cam_2d(x_, y_, side_length_, colour_spec_)
    side_half_length = abs(side_length_/2.0)  ;
    patch( ...
        [x_-side_half_length, x_-side_half_length, x_+side_half_length, x_+side_half_length] , ...
        [y_-side_half_length, y_+side_half_length, y_+side_half_length, y_-side_half_length] , ....
        colour_spec_ )  ;
    patch( ...
        [x_-side_half_length, x_-side_half_length, x_+side_half_length, x_+side_half_length] , ...
        [y_-side_half_length, y_+side_half_length, y_+side_half_length, y_-side_half_length] , ....
        colour_spec_ )  ;
end