%%----------------------------------------------------------------
%               Get length, width, CarCorners etc.
%                               O O
%                              \___/
%----------------------------------------------------------------
% BSD 3-Clause License
%
% Copyright (c) 2020, Sanchez, E., Kruber F.
% All rights reserved.
%----------------------------------------------------------------
%Input: Corners of the bounding box, image resolution, spatial resolution, LTP to PCF orientation offset, LTP to PCF linear offsets and drone height
%Output: vehicle length, width, height, mass, measurement vector for kalman filter and corners of the bounding box
function [l,w,h,m,Z,CarCorners]=lengthclass(CarCorners,resolution,meterToPx,OrientationOffset,LinearOffsets,DroneHeight,OutterHeight,InnerHeight)
global processType

%% checking if the bounding box is plausible:
%1.- are all the coordinates of the corners finite numbers? (avoid nans)
%2.- do all corners have different coordinates? (avoid duplicated corners)
if sum(sum(isfinite(CarCorners)))==8 && sum(CarCorners(1,1)~=CarCorners(2:4,1))>=2 && sum(CarCorners(1,2)~=CarCorners(2:4,2))>=2
    %Calculating the picture centroids for shifting the corners of the bounding box:
    PictureCentroids=[resolution(1)/2,resolution(2);
        resolution(1),resolution(2)/2;
        resolution(1)/2,0;
        0,resolution(2)/2;
        resolution(1)/2,resolution(2)/2];
    
    %% If the vehicle dimensions are KNOWN, the bounding box is rescaled accordingly:
    if processType == 3
        [CarCorners]=VehicleResizeLShape(CarCorners,PictureCentroids,DroneHeight,meterToPx);
    else
        [CarCorners]=vehicleresizePolygon(PictureCentroids,CarCorners,meterToPx,DroneHeight,OutterHeight,InnerHeight);
    end
    
    %% Mapping the picture centroids from Picture Coordinate Frame to Local Tangent Plane:
    PictureCentroids=PictureCentroids*meterToPx; %<--applying the spatial resolution to the picture centroids
    Rx=[1,0;0,-1]; %<--Rotation matrix of 180 deg around x axis
    for IdxRotateGCP=1:size(PictureCentroids,1)%<--Rotating the picture centroids 180 deg around x axis
        PictureCentroids(IdxRotateGCP,1:2)=transpose(transpose(Rx)*transpose(PictureCentroids(IdxRotateGCP,1:2)));
    end
    %Rotation matrix to compensate the orientation offset:
    RotationMatrix=[cos(OrientationOffset),-sin(OrientationOffset);
        sin(OrientationOffset),cos(OrientationOffset)];
    for PictureFrameIdx=1:size(PictureCentroids,1)%<--Rotating the picture centroids to compensate the orientation offset:
        PictureCentroids(PictureFrameIdx,1:2)=transpose(RotationMatrix)*transpose(PictureCentroids(PictureFrameIdx,1:2));
    end
    PictureCentroids(:,1:2)=PictureCentroids(:,1:2)-transpose(LinearOffsets); %<- Translating the picture centroids to compensate the linear offsets:
    
    %% Mapping the corners of the bounding box from Picture Coordinate Frame to Local Tangent Plane:
    CarCorners(:,3)=[1;2;3;4]; %<--Labeling the corners of the bounding box
    CarCorners(:,1:2)=CarCorners(:,1:2)*meterToPx; %<--applying the spatial resolution to the bounding box
    for IdxRotateGCP=1:size(CarCorners,1)%<--Rotating the corners of the bounding box 180 deg around x axis
        CarCorners(IdxRotateGCP,1:2)=transpose(transpose(Rx)*transpose(CarCorners(IdxRotateGCP,1:2)));
    end
    for IdxRotateGCP=1:size(CarCorners,1) %<- Rotating the corners of the bounding box to compensate the orientation offset:
        CarCorners(IdxRotateGCP,1:2)=transpose(transpose(RotationMatrix)*transpose(CarCorners(IdxRotateGCP,1:2)));
    end
    CarCorners(:,1:2)=CarCorners(:,1:2)-transpose(LinearOffsets); %<- Translating the corners of the bounding box to compensate the linear offsets:
    
    %% Identifying the inner corners of the bounding box:
    DistanceToImgCenter=nan(4,5);
    for i=1:4 %<--calculating the distance from the centroid of the image to all four corners of bounding box.
        DistanceToImgCenter(i,1)=sqrt((PictureCentroids(5,1)-CarCorners(i,1))^2+(PictureCentroids(5,2)-CarCorners(i,2))^2);
        DistanceToImgCenter(i,2)=i;
    end
    DistanceToImgCenter=sortrows(DistanceToImgCenter,1); %<--ordering the seen distances to the center from smallest to biggest
    for i=1:4 %<--calculating the distance from the CLOSEST corner to all four corners of bounding box (estimating vehicle size)
        DistanceToImgCenter(i,3)=sqrt((CarCorners(DistanceToImgCenter(1,2),1)-CarCorners(DistanceToImgCenter(i,2),1))^2+(CarCorners(DistanceToImgCenter(1,2),2)-CarCorners(DistanceToImgCenter(i,2),2))^2);
        DistanceToImgCenter(i,4:5)=[CarCorners(DistanceToImgCenter(i,2),1),CarCorners(DistanceToImgCenter(i,2),2)];
    end
    DistanceToImgCenter=sortrows(DistanceToImgCenter,3); %<--ordering the seen distances to the center from smallest to biggest
    %If the dimensions of the vehicle are NOT known, the vehicle dimensions are estimated according to:
    %1.-Seen bounding box
    %2.-European vehicle classification
    %3.-Average vehicle dimensions for each classification
    CarDictionary =[1.46,3.13,1.43,670; %<- Quad
        1.73,4.05,1.46,11702; %<- Supermini
        1.79,4.44,1.46,1335; %<- Small family car
        1.84,4.75,1.45,1502; %<- Large Family car
        1.88,4.97,1.47,1731; %<- Executive
        1.94,5.29,1.47,2051; %<- XXL
        2.45,13.6,3,23000]; %<- Truck
    CarDictionary(:,end+1)=CarDictionary(:,1)./CarDictionary(:,2);
    CarDictionary(:,end+1)=CarDictionary(:,2)./CarDictionary(:,1);
    WidthToLengthRatio=DistanceToImgCenter(2,3)/DistanceToImgCenter(3,3); %width to length ratio: aproximating vehicle dimensions according to the vehicle footprint
    Error=abs(CarDictionary(:,5)-WidthToLengthRatio); %Calculating footprint deviation for each vehicle class
    [~,I] = min(Error);%searching the vehicle class with the footprint most similar to the seen bounding box
    h=CarDictionary(I,3); % vehicle height
    m=CarDictionary(I,4); % vehicle mass
    w=DistanceToImgCenter(2,3); % vehicle width
    l=DistanceToImgCenter(3,3); % vehicle length
    
    %% Generating the measurement vector:
    % Estimating the centroid of the bounding box (x,y) coordinates in LTP
    Z(1:2,1)=[mean([DistanceToImgCenter(2,4),DistanceToImgCenter(3,4)]),mean([DistanceToImgCenter(2,5),DistanceToImgCenter(3,5)])];
    % Estimating the orientation of the bounding box in LTP
    Z(3,1)=atan2(DistanceToImgCenter(1,5)-DistanceToImgCenter(3,5),DistanceToImgCenter(1,4)-DistanceToImgCenter(3,4));
    
else  %% if there is NO meaningful measurement from the drone, no measurement vector can be generated.
    Z(1:3,1)=[nan;nan;nan];
    l=nan;
    w=nan;
    h=nan;
    m=nan;
end
end