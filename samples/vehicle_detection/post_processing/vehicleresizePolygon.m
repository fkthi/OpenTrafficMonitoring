%--------------------------------------------------------------------------
%                         VehicleResizing.m
%                               O O
%                              \___/

%                            March 2020
%
%Sanchez,E
%Kruber, F
%Resizing vehicle
%%Correcting the perspective of the vehicle. Process done IN PIXELS
%%Pulling all corners from the center towards the center.
%--------------------------------------------------------------------------
%Input: PictureCentroids,CarCorners,meterToPx,DroneHeight,OutterHeight,InnerHeight
%Output: rescaled CarCorners
function [CarCorners]=vehicleresizePolygon(PictureCentroids,CarCorners,meterToPx,DroneHeight,OutterHeight,InnerHeight)

DistanceToCenter=zeros(4,13);
for i=1:4 %% calculating the distance from all points of the polygon to the center of the image.
    DistanceToCenter(i,1)=CarCorners(i,1)-PictureCentroids(1,1);
    DistanceToCenter(i,2)=CarCorners(i,2)-PictureCentroids(2,2);
    DistanceToCenter(i,3)=sqrt(DistanceToCenter(i,1)^2+DistanceToCenter(i,2)^2);
    DistanceToCenter(i,4)=i;
    DistanceToCenter(i,5)=CarCorners(i,1);
    DistanceToCenter(i,6)=CarCorners(i,2);
end
DistanceToCenter=sortrows(DistanceToCenter,3); %% ordering the corners according to distance to the center of the picture
for i=2:4 %% Calculating distance from the corner closes to the center to all others
    DistanceToCenter(i,7)=sqrt((DistanceToCenter(i,5)-DistanceToCenter(1,5))^2+(DistanceToCenter(i,6)-DistanceToCenter(1,6))^2);
end
DistanceToCenter=sortrows(DistanceToCenter,7); %%ordering according to the distance to the closest corner to the center

DroneHeightInPix=DroneHeight/meterToPx; %% drone height in pixels

OutterHeightInPx=OutterHeight/meterToPx;
InnerHeightInPx=InnerHeight/meterToPx;



for i=[1,3] %% The side closest to the center is scaled with inner height
    DistanceToCenter(i,8)=(InnerHeightInPx*DistanceToCenter(i,1))/DroneHeightInPix; %% computing correctION in X axis
    DistanceToCenter(i,9)=(InnerHeightInPx*DistanceToCenter(i,2))/DroneHeightInPix; %% computing correctION in Y axis
    DistanceToCenter(i,10)=DistanceToCenter(i,1)-DistanceToCenter(i,8); %% Adding correction to distance to center in X axis
    DistanceToCenter(i,11)=DistanceToCenter(i,2)-DistanceToCenter(i,9); %% Adding correction to distance to center in Y axis
    DistanceToCenter(i,12)=DistanceToCenter(i,10)+PictureCentroids(1,1); %% computing correctED coordinate in X axis
    DistanceToCenter(i,13)=DistanceToCenter(i,11)+PictureCentroids(2,2); %% computing correctED coordinate in Y axis
end

for i=[2,4] %% The side closest to the center is scaled with outer height
    DistanceToCenter(i,8)=(OutterHeightInPx*DistanceToCenter(i,1))/DroneHeightInPix; %% computing correctION in X axis
    DistanceToCenter(i,9)=(OutterHeightInPx*DistanceToCenter(i,2))/DroneHeightInPix; %% computing correctION in Y axis
    DistanceToCenter(i,10)=DistanceToCenter(i,1)-DistanceToCenter(i,8); %% Adding correction to distance to center in X axis
    DistanceToCenter(i,11)=DistanceToCenter(i,2)-DistanceToCenter(i,9); %% Adding correction to distance to center in Y axis
    DistanceToCenter(i,12)=DistanceToCenter(i,10)+PictureCentroids(1,1); %% computing correctED coordinate in X axis
    DistanceToCenter(i,13)=DistanceToCenter(i,11)+PictureCentroids(2,2); %% computing correctED coordinate in Y axis
end

DistanceToCenter=sortrows(DistanceToCenter,4); %% ordering the corners according to distance to the center of the picture
CarCorners(1:4,1:2)=DistanceToCenter(1:4,12:13);

end