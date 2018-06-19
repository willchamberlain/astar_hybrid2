function SO3__ = se3_to_so3(SE3_)
    if size(SE3_,1) == 4 && size(SE3_,2) == 4
        SO3__ = SE3_(1:3,1:3,:)  ;
    elseif size(SE3_,2) == 4 && size(SE3_,3) == 4
        SO3__ = SE3_(:,1:3,1:3)  ;        
    end
end