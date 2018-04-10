%{
Generates noise to add to pixel coordinates.  Currently hardcoded to generate Gaussian noise
%}
function points_2D_noise__ = camera_extrinsics__generate_noise_for_points_2D(num_points_, Gaussian_magnitude_, Gaussian_mean_, Gaussian_sd_)
    points_2D_noise__ = normrnd( Gaussian_mean_, Gaussian_sd_, 1, num_points_ ) ;
    points_2D_noise__ = points_2D_noise__  .* (Gaussian_magnitude_/3);  % Gaussian has magnitude approx +/- 3
%     points_2D_noise__ = 1/Gaussian_sd_ * vl_gaussian( (      (rand(1,num_points_) -0.5) *3.0  *1/Gaussian_sd_    ) + Gaussian_mean_) ;
%     points_2D_noise__ = Gaussian_magnitude_ * points_2D_noise__;
end