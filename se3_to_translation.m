function translation__ = se3_to_translation(SE3_)
    if size(SE3_,1) == 4 && size(SE3_,2) == 4
        translation__ = SE3_(1:3,4,:)  ;
    elseif size(SE3_,2) == 4 && size(SE3_,3) == 4
        translation__ = SE3_(:,1:4,4)'  ;       
    end
    
end