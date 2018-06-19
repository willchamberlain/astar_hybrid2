function plot_point3d_SO3(SO3_, varargin)
    size_ = size(SO3_)  ;
    if [4,4] ~= size_(1,2)
        display('Expecting 4x4xN')  ;   
        return  ;
    end
    plot3_rows( SO3_(1:3,4,:) ,varargin) ; 
end