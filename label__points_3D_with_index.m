function label__points_3D_with_index(points_3D_)
    for ii_ = 1: size(points_3D_, 2)
        text(points_3D_(1,ii_),   points_3D_(2,ii_),   points_3D_(3,ii_),   sprintf('%d',ii_)  );
    end
end