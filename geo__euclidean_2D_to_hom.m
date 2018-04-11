%{
Converts 2D Euclidean coordinate vectors to 3D homogeneous coordinate vectors.
Default / no varargin - is column-major:  [ x ; y ]  --> [ x ; y ; 1 ]
If varagin(1,2) is 'rows' - case-insensitive : execution is row-major: [ x y ] --> [ x y 1 ]
%}
function homogeneous_2D__ = geo__euclidean_2D_to_hom(euclidean_2D_, varargin)
    if size(varargin,2) > 0 && strcmpi('rows',varargin(1,1))
        if size(euclidean_2D_,2) ==2
            homogeneous_2D__ = horzcat(  euclidean_2D_  ,  ones(size(euclidean_2D_,1),1) );      
        elseif size(euclidean_2D_,2) ==3
            if euclidean_2D_(:,3) == ones(size(euclidean_2D_,1),1)
                display(sprintf('geo__euclidean_2D_to_hom( (%d,%d) , %s) : appears to be homogeneous already',size(euclidean_2D_,1),size(euclidean_2D_,2),char(varargin(1,1))));
                homogeneous_2D__ = euclidean_2D_;
            else
                display(sprintf('geo__euclidean_2D_to_hom( (%d,%d) , %s) is not valid',size(euclidean_2D_,1),size(euclidean_2D_,2),char(varargin(1,1))));
                homogeneous_2D__ = [];
            end
        elseif    size(euclidean_2D_,2) < 2   || size(euclidean_2D_,2) > 3    
            display(sprintf('geo__euclidean_2D_to_hom( (%d,%d) , %s) is not valid',size(euclidean_2D_,1),size(euclidean_2D_,2),char(varargin(1,1))));
            homogeneous_2D__ = [];
        end
    else
        homogeneous_2D__ = geo__euclidean_2D_to_hom(euclidean_2D_', 'rows');
        homogeneous_2D__ = homogeneous_2D__';
    end
end