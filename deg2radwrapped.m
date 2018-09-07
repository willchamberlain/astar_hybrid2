function rads_wrapped__ = deg2radwrapped(degrees_)    
    rads = deg2rad(degrees_)  ;
    if rads >= 0
        rads_wrapped__ = mod(rads,2*pi) ;
    else
        rads_wrapped__ = mod(rads,-2*pi) ;
    end
end