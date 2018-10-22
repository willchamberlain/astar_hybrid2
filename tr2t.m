function translation__ = tr2t(transform_SE3_)
    [r,t] = tr2rt(transform_SE3_)  ;
    translation__ = t  ;
end 