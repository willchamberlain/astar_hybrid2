addpath( '/mnt/nixbig/ownCloud/project_code/' )

%%


imaqhwinfo

% webcam_object = imaq.VideoDevice('linuxvideo',1,'BGR24_1920×1080')
webcam_object = imaq.VideoDevice
[frame metadata] = step(webcam_object);
imshow(frame);

webcam_object.Device
webcam_object.DeviceProperties

webcam_object.DeviceProperties('WhiteBalanceMode')

webcam_object.ReturnedColorSpace
webcam_object.ReturnedDataType
webcam_object.ROI

%%  green box as ID tag
fig_rgb_h = figure_named('RGB')
fig_hsv_h = figure_named('HSV')

while true    
    [frame metadata] = step(webcam_object);
    figure(fig_rgb_h);
    clf;
    masked_rgb = frame(:,:,1) > 0.1 & frame(:,:,1) < 0.2 & frame(:,:,2) > 0.35 & frame(:,:,1) < 0.55 & frame(:,:,3) > 0.5 & frame(:,:,3) < 0.7  ;
    %  idisp(masked_rgb);
    idisp(frame) ;
    figure(fig_hsv_h);
    clf;
    frame_hsv = rgb2hsv(frame);
    masked_hsv = frame_hsv(:,:,1) > 0.5 & frame_hsv(:,:,1) < 0.6 & frame_hsv(:,:,2) > 0.7 & frame_hsv(:,:,2) < 0.95 & frame_hsv(:,:,3) > 0.5 & frame_hsv(:,:,3) < 0.8  ;
    idisp(masked_hsv & masked_rgb);
end

    
    masked_rgb = frame(:,:,:) > 0.5  ;
    figure  ; 
    idisp(masked_rgb)  ;
    
    rgb_rgb = cat( 3,  frame(:,:,1)./frame(:,:,2)  ,  frame_hsv(:,:,2)  ,  frame(:,:,3)./frame(:,:,2) ) ;
    figure  ; 
    idisp(rgb_rgb)  ;

    %  GuiaBot / PatrolBot white 
    masked =     ...
        frame_hsv(:,:,1) > 0.5 & frame_hsv(:,:,1) < 0.6 & frame_hsv(:,:,2) > 0.2 & frame_hsv(:,:,2) < 0.4  ...
        &  ...
        frame(:,:,1)./frame(:,:,2) > 0.7 & frame(:,:,1)./frame(:,:,2) < 1.0  & ...
        frame(:,:,3)./frame(:,:,2) > 1.1 & frame(:,:,3)./frame(:,:,2) < 1.4  & ...
        frame_hsv(:,:,3) > 0.85;
    figure; idisp(masked)
    figure; idisp(frame_hsv(:,:,3))

subplot(1,2,1);  imshow(frame) ;
subplot(1,2,2); imshow(rgb2lab(frame)) ;
    
A = frame_hsv;
N=1000;
[L,NumLabels] = superpixels(A,N, 'NumIterations', 20, 'Compactness', 10)    ;
figure
BW = boundarymask(L);
imshow(imoverlay(A,BW,'cyan'),'InitialMagnification',67)
outputImage = zeros(size(A),'like',A);
idx = label2idx(L);
numRows = size(A,1);
numCols = size(A,2);
for labelVal = 1:N
    redIdx = idx{labelVal};
    greenIdx = idx{labelVal}+numRows*numCols;
    blueIdx = idx{labelVal}+2*numRows*numCols;
    outputImage(redIdx) = mean(A(redIdx));
    outputImage(greenIdx) = mean(A(greenIdx));
    outputImage(blueIdx) = mean(A(blueIdx));
end    
figure
% imshow(outputImage,'InitialMagnification',67)    
imshow(outputImage)  ;
    
imshow(frame_hsv)

%%
xyxy=[1374,798 , 1546, 750]  ;

frame_hsv_patch=frame_hsv(  small_large_range(xyxy(2),xyxy(4)) ,  small_large_range(xyxy(1),xyxy(3)),  :  ) ; 
subplot(2,3,1); imshow(frame_hsv)
subplot(2,3,2); imshow(frame_hsv_patch) ;
subplot(2,3,4); surf(frame_hsv_patch(:,:,1)) ;
subplot(2,3,5); histogram2(frame_hsv_patch(:,:,1), frame_hsv_patch(:,:,2))
subplot(2,3,6); histogram2(frame_hsv_patch(:,:,2), frame_hsv_patch(:,:,3))
subplot(2,3,5); imshow( cat(3, ...
    frame_hsv_patch(:,:,1)./frame_hsv_patch(:,:,2)  ,  ...
    frame_hsv_patch(:,:,2)./frame_hsv_patch(:,:,3)  ,  ...
    zeros(size(frame_hsv_patch,1), size(frame_hsv_patch,2) )  ...
    )  ) ;
subplot(2,3,5);  surf( (frame_hsv_patch(:,:,1)./frame_hsv_patch(:,:,2)).*(frame_hsv_patch(:,:,2)./frame_hsv_patch(:,:,3)) )

xyxy2=[1000,1000,1800,200] ;
frame_hsv_patch_big = frame_hsv(  small_large_range(xyxy2(2),xyxy2(4)) ,  small_large_range(xyxy2(1),xyxy2(3)),  :  ) ;
subplot(2,3,5);  imshow( (frame_hsv(:,:,1)./frame_hsv(:,:,2)).*(frame_hsv(:,:,2)./frame_hsv(:,:,3)) )

[N,XEDGES,YEDGES] =  histcounts2(frame_hsv_patch(:,:,1), frame_hsv_patch(:,:,2))  ;
[i,j] = find(N > 100)
size(XEDGES)
size(YEDGES)
size(N)
[i,j]=ind2sub(size(N), find(N>100))
h_set = XEDGES(i) ;  h_range = [ min(h_set), max(h_set) ]  
s_set = YEDGES(j) ;  s_range = [ min(s_set), max(s_set) ]  
imshow( h>=min(h_set) & h<=max(h_set)  &  s>=min(s_set) & s<=max(s_set) )

[  ceil(567/size(XEDGES,2)) , mod(567,size(XEDGES,2))  ]


[  ceil(28*32/  (size(YEDGES,2)-1)  ) , mod(32*28, (size(XEDGES,2)-1)   )  ]

idisp(frame_hsv)

h=frame_hsv(:,:,1)  ;
s=frame_hsv(:,:,2)  ;
v=frame_hsv(:,:,3)  ;
figure; imshow( double(cat(3,h>0.8,s>0.4&s<0.8, v>0.2 & v<0.5) ))

%%   Statistics - simple histogram over a patch - patch is currently delimited by hand, could be by movement  
xyxy=[1374,798 , 1546, 750]  ;
frame_hsv_patch=frame_hsv(  small_large_range(xyxy(2),xyxy(4)) ,  small_large_range(xyxy(1),xyxy(3)),  :  ) ; 
[N,XEDGES,YEDGES] =  histcounts2(frame_hsv_patch(:,:,1), frame_hsv_patch(:,:,2))  ;
[i,j] = find(N > 100)
size(XEDGES)
size(YEDGES)
size(N)
[i,j]=ind2sub(size(N), find(N>100))
h_set = XEDGES(i) ;  h_range = [ min(h_set), max(h_set) ]  
s_set = YEDGES(j) ;  s_range = [ min(s_set), max(s_set) ]  
imshow( h>=min(h_set) & h<=max(h_set)  &  s>=min(s_set) & s<=max(s_set) )


%%

%  Pioneer red :
h=frame_hsv(:,:,1)  ;
s=frame_hsv(:,:,2)  ;
v=frame_hsv(:,:,3)  ;
figure; imshow( double(cat(3,h>0.8,s>0.4&s<0.8, v>0.2 & v<0.5) ))


%  GuiaBot / PatrolBot white 
masked =     ...
    frame_hsv(:,:,1) > 0.5 & frame_hsv(:,:,1) < 0.6 & frame_hsv(:,:,2) > 0.2 & frame_hsv(:,:,2) < 0.4  ...
    &  ...
    frame(:,:,1)./frame(:,:,2) > 0.7 & frame(:,:,1)./frame(:,:,2) < 1.0  & ...
    frame(:,:,3)./frame(:,:,2) > 1.1 & frame(:,:,3)./frame(:,:,2) < 1.4  & ...
    frame_hsv(:,:,3) > 0.85;
figure; idisp(masked)
figure; idisp(frame_hsv(:,:,3))


%%
%  preview is faster than step(webcam_object) --> imshow
webcam_object.preview



%%

while true
    [frame metadata] = step(webcam_object);
    imshow(frame);
    pause(0.10)
end



%%
webcam_object.closepreview
webcam_object.release

%%
imwrite(frame,'/mnt/nixbig/temp/S11_lab_floor_.png')

%%
clear('im_array')
delete(im_array)

im=imread('/mnt/nixbig/temp/S11_lab_floor.png')  ;
size( im )
im_array(size( im ) , 1 ) = im
im_array = im ;
size(im_array)

im_ = imread('/mnt/nixbig/temp/S11_lab_floor_.png')  ;
size( im_ )
size( im_array )
im_array(size( im ) , 2 ) = im_  ;
im_array(:,:,: , 2 ) = im_  ;


imshow( im )
idisp( im )
fig_handle = []
if true
    if exist(fig_handle); 
        display('here'); 
        figure(fig_handle); 
    else
        display('not here');
        fig_handle = figure_named('Processed')  ;
    end
    
    1536        *2304         *  3
    im_processed = im(im(:,:,1) > 30 & im(:,:,1)<70);
    size( im_processed )
    reshape(im_processed, size(im))
    idisp(im_processed)
end

size(im(:,:,1)<70)
size(  im( im(:,:,1)<70)  )
size(  im  )
1536    *   2304
idisp( im( im(:,:,1)<70) )

idisp( im( im(:,:,1)<70 )  )

idisp(  ...
im(:,:,1) > 30 & im(:,:,1)<70  ...
&  ...
im(:,:,2) > 40 & im(:,:,2)<900  ...
&  ...
im(:,:,3) > 70 & im(:,:,3)<120  ...
)




