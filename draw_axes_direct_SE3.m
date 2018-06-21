function draw_axes_direct_SE3(SE3_, text_, scale_)
    draw_axes_direct(SE3_(1:3,1:3), SE3_(1:3,4), text_, scale_)
end