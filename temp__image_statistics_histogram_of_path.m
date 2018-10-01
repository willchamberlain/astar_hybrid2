function  [pixel_mask__ , h_range__ , s_range__] = temp__image_statistics_histogram_of_path( hsv_image_,  x1_, y1_,  x2_, y2_ , thresh_)

% xyxy=[1374,798 , 1546, 750]  ;
xyxy=[  x1_, y1_,  x2_, y2_  ]  ;
frame_hsv = hsv_image_  ;
h = hsv_image_(:,:,1);
s = hsv_image_(:,:,2);


frame_hsv_patch=frame_hsv(  small_large_range(xyxy(2),xyxy(4)) ,  small_large_range(xyxy(1),xyxy(3)),  :  ) ; 
[N,XEDGES,YEDGES] =  histcounts2(frame_hsv_patch(:,:,1), frame_hsv_patch(:,:,2))  ;
rates = sort( reshape(N, size(N,1)*size(N,2), 1) , 'descend')  ;
cum_sum_rates = cumsum(rates)  ;
pop_size=size(frame_hsv_patch,1)*size(frame_hsv_patch,2)  ;
rate_thresh_idx = min(find(cum_sum_rates> (pop_size*(0.3))))  ;
rate_thresh = rates(rate_thresh_idx)  ;
display(rate_thresh_idx)
display(rate_thresh )

[i,j] = find(N > thresh_)  ;
    size(XEDGES)  ;
    size(YEDGES)  ;
    size(N)  ;
    [i,j]=ind2sub(size(N), find(N>thresh_))  ;
h_set = XEDGES(i) ;  h_range__ = [ min(h_set), max(h_set) ]  ;
s_set = YEDGES(j) ;  s_range__ = [ min(s_set), max(s_set) ]  ;
    % imshow( h>=min(h_set) & h<=max(h_set)  &  s>=min(s_set) & s<=max(s_set) )
pixel_mask__ = h>=min(h_set) & h<=max(h_set)  &  s>=min(s_set) & s<=max(s_set)   ;
end