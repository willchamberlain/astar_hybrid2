function translation_vec_rdf = flu_to_rdf(translation_vec_flu)
    change_of_basis_flu_to_rdf = ...
        [   0 0 1 ; 
           -1 0 0 ; 
           0 -1 0 ]';
    change_of_basis_flu_to_rdf_hom = ...
        [    0  0  1   0; 
           -1  0  0    0; 
           0  -1  0    0;
           0   0  0    1]';
    if 4 == size(translation_vec_flu,1)  
        translation_vec_rdf =  change_of_basis_flu_to_rdf_hom* translation_vec_flu;        
    else
        translation_vec_rdf =  change_of_basis_flu_to_rdf* translation_vec_flu;
    end
end


