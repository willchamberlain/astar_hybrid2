function translation_xyz__ = geom__transform_to_translation(transform_SE3_)
    [R, translation_xyz__] = tr2rt(transform_SE3_);    
end