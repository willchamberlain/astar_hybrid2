function translation_vec_flu = rdf_to_flu(translation_vec_rdf)
    change_of_basis_rdf_to_flu = ...
        [   0 0 1 ; 
           -1 0 0 ; 
           0 -1 0 ];
    translation_vec_flu =  change_of_basis_rdf_to_flu* translation_vec_rdf;
end