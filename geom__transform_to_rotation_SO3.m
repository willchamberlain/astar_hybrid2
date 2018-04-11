function rotation_SO3__ = geom__transform_to_rotation_SO3(transform_SE3_)
    [rotation_SO3__,t] = tr2rt(transform_SE3_);   
end