
function m = camera_resolution(radius)
%{
    2D camera resolution values.
Example:
cam_res_ = camera_resolution(50);
surf(cam_res_) 
%}
    w = radius ;
    [x,y] = meshgrid(-w:w, -w:w) ;
    size(-(x.^2 + y.^2))    
%     figure; surf(x,y, exp( -(x.^2 + y.^2)) ) ; % axis equal
%     sigma = 5;
%     figure; surf(x,y,  1/(2*pi*sigma^2) * exp( -(x.^2 + y.^2)/2/sigma^2)  )  ;
    m = 1-exp(   (x.^2 + y.^2)./max(max(  (x.^2 + y.^2) ))   );
    m = m./min(min(m)) ;
    m = 1-m;
    m (m(:,:) < 0.6) = 0;
    m = m - 0.6;
    m (m(:,:) < 0) = 0;
    m = m.*( 1 / max(max(m))  ) ;
%     m = m.*0.5;
%     m = 0.5 - m;
%     imshow(m);
%      figure; surf(x,y, m ) ;  % axis equal
end