
function norm_2__ = norm_2 (A_,dim_)
% 2-norm of rows/columns of A_ along the dimension dim_  
%  (if dim_ is 1, norms along columns; if dim_ is 2, norms along rows, etc.).
%
%
%
    norm_2__ = sqrt( sum( real(A_).*conj(A_) , dim_) )  ;
end