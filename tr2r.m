function rotlation__ = tr2r(transform_SE3_)
    [r,t] = tr2rt(transform_SE3_)  ;
    rotlation__ = r  ;
end 