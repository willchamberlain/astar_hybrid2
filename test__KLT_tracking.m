videoPlayer = vision.VideoPlayer('Position',[100,100,500,400]);
foregroundMaskPlayer = vision.VideoPlayer('Position',[500,400,500,400]);
videoReader = imaq.VideoDevice

%%
 colorImage  = step(videoReader);
     
points = detectMinEigenFeatures(rgb2gray(colorImage)) ; %,'ROI',objectRegion);
pointImage = insertMarker(colorImage,points.Location,'+','Color','white');
step(videoPlayer,pointImage);


tracker = vision.PointTracker('MaxBidirectionalError',1);
initialize(tracker,points.Location,colorImage);

while true
     colorImage  = step(videoReader);
     [points,validity,scores] = step(tracker, colorImage);
%      detectedLocation = step(blobAnalyzer,foregroundMask);
%      isObjectDetected = size(detectedLocation, 1) > 0;
% 
%        if isObjectDetected 
%          label = 'Detected';
%          circle = [detectedLocation(1,:), 5];
%        else
%          trackedLocation = predict(kalmanFilter);
%          label = 'Not detected';
%          circle = [10,10, 5];
%        end
%      colorImage = insertObjectAnnotation(colorImage,'circle',circle,label,'Color','red');
%      step(videoPlayer,colorImage);
%      step(foregroundMaskPlayer,foregroundMask);
%      pause(0.1)
      out = insertMarker(colorImage,points(validity, :),'+');
      step(videoPlayer,out);
   end
%%
videoReader.reset
%%
% Release resources.
release( tracker )
%
release(videoPlayer);
release(foregroundMaskPlayer);
release(videoReader);   