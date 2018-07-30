function B__ = sort_into_rows_of_matrix(A_)
    B__ = reshape(sort(A_(:)), fliplr(size(A_)))'  ;
end