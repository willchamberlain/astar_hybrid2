%{
Converts 3D Euclidean coordinate vectors to 3D homogeneous coordinate vectors.
Default / no varargin - is column-major:  [ x ; y ; x ]  --> [ x ; y ; z ; 1 ]
If varagin(1,2) is 'rows' - case-insensitive : execution is row-major: [ x y z ] --> [ x y z 1 ]
%}
function homogeneous_3D__ = geo__euclidean_3D_to_hom(euclidean_3D_, varargin)
    if size(varargin,2) > 0 && strcmpi('rows',varargin(1,1))
        if size(euclidean_3D_,2) ==3
            homogeneous_3D__ = horzcat(  euclidean_3D_  ,  ones(size(euclidean_3D_,1),1) );      
        elseif size(euclidean_3D_,2) ==4
            if euclidean_3D_(:,4) == ones(size(euclidean_3D_,1),1)
                display(sprintf('geo__euclidean_3D_to_hom( (%d,%d) , %s) : appears to be homogeneous already',size(euclidean_3D_,1),size(euclidean_3D_,2),char(varargin(1,1))));
                homogeneous_3D__ = euclidean_3D_;
            else
                display(sprintf('geo__euclidean_3D_to_hom( (%d,%d) , %s) is not valid',size(euclidean_3D_,1),size(euclidean_3D_,2),char(varargin(1,1))));
                homogeneous_3D__ = [];
            end
        elseif    size(euclidean_3D_,2) < 3   || size(euclidean_3D_,2) > 4         
            display(sprintf('geo__euclidean_3D_to_hom( (%d,%d) , %s) is not valid',size(euclidean_3D_,1),size(euclidean_3D_,2),char(varargin(1,1))));
            homogeneous_3D__ = [];
        end
    else
%         homogeneous_3D__ =  [euclidean_3D_; ones(1,size(euclidean_3D_,2))];
        homogeneous_3D__ = geo__euclidean_3D_to_hom(euclidean_3D_', 'rows');
        homogeneous_3D__ = homogeneous_3D__';
    end
end