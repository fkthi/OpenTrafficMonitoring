%%----------------------------------------------------------------
%                           Resizing vehicle
%               Correcting the perspective of the vehicle. 
%                       Process done IN PIXELS
%                               O O
%                              \___/
%----------------------------------------------------------------
% BSD 3-Clause License
%
% Copyright (c) 2020, Sanchez, E., Kruber F.
% All rights reserved.
%----------------------------------------------------------------
%Input: Corners of the bounding box, picture centroids, drone height and spatial resolution
%Output: rescaled corners of the bounding box
function [CarCorners]=VehicleResizeLShape(CarCorners,PictureCentroids,DroneHeight,meterToPx)

DistanceToCenter=zeros(4,14);
for i=1:4 %% calculating the distance from all points of the polygon to the center of the image.
    DistanceToCenter(i,1)=CarCorners(i,1)-PictureCentroids(end,1); %<- change in the x axis
    DistanceToCenter(i,2)=CarCorners(i,2)-PictureCentroids(end,2); %<- change in the y axis
    DistanceToCenter(i,3)=sqrt(DistanceToCenter(i,1)^2+DistanceToCenter(i,2)^2); % euclidean distance from image center to each corner
    DistanceToCenter(i,4)=i; % saving corner label
    DistanceToCenter(i,5)=CarCorners(i,1); % Saving corner x coordinate
    DistanceToCenter(i,6)=CarCorners(i,2); % Saving corner y coordinate
end
DistanceToCenter=sortrows(DistanceToCenter,3); %<- ordering the corners according to distance to the center of the picture
for i=2:4 %% Calculating distance from the corner closest to the center to all others
    DistanceToCenter(i,7)=sqrt((DistanceToCenter(i,5)-DistanceToCenter(1,5))^2+(DistanceToCenter(i,6)-DistanceToCenter(1,6))^2);
end
DistanceToCenter=sortrows(DistanceToCenter,7); %<- ordering according to the distance to the closest corner to the center

DroneHeightInPix=DroneHeight/meterToPx; %<- drone height in pixels

VehicleInnerHeights=[0.15;0.75]; %<- vehicle ground clearance and height of the vehicle shoulder
%Handover radius: Are we seeing the middle or the floor of the car?:
InnerHandoverRadiusInMeters=DroneHeight*(0.061889/0.65);
InnerHandoverRadiusInPx=InnerHandoverRadiusInMeters/meterToPx;
% VehicleOutterHeights=[0.75;1.2080;1.434];
%Handover radius Middle: Are we seeing the window or the handle of the car?
% OuterHandoverRadiusMiddleInMeters=DroneHeight*(0.272/0.408);
% OuterHandoverRadiusMiddleInPx=OuterHandoverRadiusMiddleInMeters/meterToPx;
%Handover radius Top: Are we seeing the roof of the car?
% OuterHandoverRadiusTopInMeters=DroneHeight*(1.268/1.434);
% OuterHandoverRadiusTopInPx=OuterHandoverRadiusTopInMeters/meterToPx;
%% Determining heights for the INNER corners:
for i=1 
    if DistanceToCenter(i,3)>InnerHandoverRadiusInPx
        DistanceToCenter(i,8)=VehicleInnerHeights(1);
    else
        DistanceToCenter(i,8)=VehicleInnerHeights(2);
    end
end

for i=1 %<- Shifting the closest corneres towards the center
    DistanceToCenter(i,9)=(DistanceToCenter(i,8)*DistanceToCenter(i,1))/DroneHeightInPix; %<- computing correctION in X axis
    DistanceToCenter(i,10)=(DistanceToCenter(i,8)*DistanceToCenter(i,2))/DroneHeightInPix; %<- computing correctION in Y axis
    DistanceToCenter(i,11)=DistanceToCenter(i,1)-DistanceToCenter(i,9); %<- Adding correction to distance to center in X axis
    DistanceToCenter(i,12)=DistanceToCenter(i,2)-DistanceToCenter(i,10); %<- Adding correction to distance to center in Y axis
    DistanceToCenter(i,13)=DistanceToCenter(i,11)+PictureCentroids(1,1); %<- computing correctED coordinate in X axis
    DistanceToCenter(i,14)=DistanceToCenter(i,12)+PictureCentroids(2,2); %<- computing correctED coordinate in Y axis
end

VehicleWidthInPix=1.842/meterToPx; %<- vehicle width in pixels
DistanceToCenter(2,8)=VehicleWidthInPix/DistanceToCenter(2,7); %<- rescale factor of the width
VehicleLengthInPix=4.725/meterToPx; %<- vehicle length in pixels
DistanceToCenter(3,8)=VehicleLengthInPix/DistanceToCenter(3,7); %<- rescale factor of the length
DistanceToCenter(4,8)=VehicleLengthInPix/DistanceToCenter(3,7); %<- rescale factor of the length

for i=2:4 %<- Scaling car with known size
    DistanceToCenter(i,9)=(DistanceToCenter(i,1)-DistanceToCenter(1,1))*DistanceToCenter(i,8); %<- computing correctED size in pixels in X axis
    DistanceToCenter(i,10)=(DistanceToCenter(i,2)-DistanceToCenter(1,2))*DistanceToCenter(i,8); %<- computing correctED size in pixels in in Y axis
    DistanceToCenter(i,11)=DistanceToCenter(i,9)+DistanceToCenter(1,1); %<- Adding correctION to distance to center in X axis
    DistanceToCenter(i,12)=DistanceToCenter(i,10)+DistanceToCenter(1,2); %<- Adding correctION to distance to center in Y axis
    DistanceToCenter(i,13)=DistanceToCenter(i,11)+PictureCentroids(1,1); %<- computing correctED coordinate in X axis
    DistanceToCenter(i,14)=DistanceToCenter(i,12)+PictureCentroids(2,2); %<- computing correctED coordinate in Y axis
end

DistanceToCenter=sortrows(DistanceToCenter,4); %<- ordering the corners according to distance to the center of the picture
CarCorners(1:4,1:2)=DistanceToCenter(1:4,13:14); %<- Saving the rescaled corners of the bounding box
end