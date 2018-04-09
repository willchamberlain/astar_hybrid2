function goodness_of_models_ = goodness_of_models(reprojected_errs_euc_hist , reprojected_error_threshold)
    goodness_of_models_ = sum ( reprojected_errs_euc_hist(:,:) < reprojected_error_threshold , 2 )  ;
end