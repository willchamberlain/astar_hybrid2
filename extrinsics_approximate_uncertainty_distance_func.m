function uncertainty__ = extrinsics_approximate_uncertainty_distance_func(distances_)
    X = distances_ ;
    Y = exp(X/1.1)/300;
    y_lim = Y;  y_lim(y_lim>1)=1; y_lim(y_lim<0)=0;
    uncertainty__ = y_lim  ;
    
    %     prob_y = 1-Y;
    %     prob_y_lim = prob_y;  prob_y_lim(prob_y_lim>1)=1; prob_y_lim(prob_y_lim<0)=0;
    %     uncertainty__ = prob_y_lim ; 
end