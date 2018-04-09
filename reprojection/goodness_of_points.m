function goodness_of_points_ = goodness_of_points(reprojected_errs_euc_hist , reprojected_error_threshold)
    goodness_of_points_  = sum ( reprojected_errs_euc_hist(:,:) < reprojected_error_threshold , 1 );
end