videoPlayer = vision.VideoPlayer('Position',[100,100,500,400],'Name','Colour');
hsvPlayer = vision.VideoPlayer('Position',[500,400,500,400],'Name','HSV');
maskPlayer = vision.VideoPlayer('Position',[1000,400,500,400],'Name','HSV');
videoReader = imaq.VideoDevice

%%   plain colour blob detector 

%%
while true    
    colorImage  = step(videoReader)  ;
    hsvImage = rgb2hsv(colorImage)  ;
    masked = zeros(size(hsvImage))  ;
    masked( ...
            hsvImage(:,:,1) > 0.015 & hsvImage(:,:,1) < 0.04 ...
        & hsvImage(:,:,2) > 0.750 & hsvImage(:,:,2) < 1.10 ...
        & hsvImage(:,:,3) > 0.200 & hsvImage(:,:,3) < 0.75 ...
    ) = 1  ;
    step(videoPlayer,colorImage)  ;
    step(hsvPlayer,hsvImage)  ;
    step(maskPlayer, masked)  ;
    pause(0.1) ;
end
%%
videoReader.reset
%%
% Release resources.
%
release(videoPlayer);
release(hsvPlayer);
release(videoReader);   

