function [CarCornersInPicture]=BackToPixel(CarCornersKF,MeterToPx,OrientationOffset,LinearOffsets)

CarCornersInPicture(:,1:2)=CarCornersKF(:,1:2)+transpose(LinearOffsets); %<- Translating the corners of the bounding box to compensate the linear offsets:

%Rotation matrix to compensate the orientation offset:
RotationMatrix=[cos(OrientationOffset),-sin(OrientationOffset);
    sin(OrientationOffset),cos(OrientationOffset)];
for IdxRotateGCP=1:size(CarCornersInPicture,1) %<- Rotating the corners of the bounding box to compensate the orientation offset:
    CarCornersInPicture(IdxRotateGCP,1:2)=transpose((RotationMatrix)*transpose(CarCornersInPicture(IdxRotateGCP,1:2)));
end

Rx=[1,0;0,-1]; %<--Rotation matrix of 180 deg around x axis
for IdxRotateGCP=1:size(CarCornersInPicture,1)%<--Rotating the corners of the bounding box 180 deg around x axis
    CarCornersInPicture(IdxRotateGCP,1:2)=transpose((Rx)*transpose(CarCornersInPicture(IdxRotateGCP,1:2)));
end

CarCornersInPicture(:,1:2)=CarCornersInPicture(:,1:2)/MeterToPx; %<--applying the spatial resolution to the bounding box

end