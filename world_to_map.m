function [u__,v__] = world_to_map(x_,y_) 
    u__ = (x_ * 4.5) + 160 ;
    v__ = (-1.0 * y_ * 4.5) + 105 ;
end