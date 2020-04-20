%%----------------------------------------------------------------
% Compute Orientation and Linear Offsets based on the VideoMeta 
%       Only relevant for Benchmark with Reference Sensor
%                               O O
%                              \___/
%----------------------------------------------------------------
% BSD 3-Clause License
%
% Copyright (c) 2020, Kruber F., Sanchez, E.
% All rights reserved.
%----------------------------------------------------------------

function [OrientationOffset, LinearOffsets, CalibrationErrors, GCP] = ComputeOffsets(meterToPx, GCP)

Rx=[1,0;0,-1]; %%<-- Rotation matrix of 180 deg around x axis
for IdxRotateGCP=1:size(GCP,1)
    GCP(IdxRotateGCP,3:4)=transpose(transpose(Rx)*transpose(GCP(IdxRotateGCP,3:4)));
end

GCP(1:3,5:6)=GCP(1:3,3:4)*meterToPx;

RefLTP=[atan2(GCP(3,2)-GCP(2,2),GCP(3,1)-GCP(2,1)),atan2(GCP(1,2)-GCP(2,2),GCP(1,1)-GCP(2,1)),atan2(GCP(3,2)-GCP(1,2),GCP(3,1)-GCP(1,1))];
DroneLTP=[atan2(GCP(3,6)-GCP(2,6),GCP(3,5)-GCP(2,5)),atan2(GCP(1,6)-GCP(2,6),GCP(1,5)-GCP(2,5)),atan2(GCP(3,6)-GCP(1,6),GCP(3,5)-GCP(1,5))];
OrientationOffset=mean(DroneLTP-RefLTP);

RotationMatrix=[cos(OrientationOffset),-sin(OrientationOffset);
    sin(OrientationOffset),cos(OrientationOffset)]; %<-- Rotation matrix of OrientationOffset degrees around Z axis
GCP(1,7:8)=transpose(RotationMatrix)*transpose(GCP(1,5:6));
GCP(2,7:8)=transpose(RotationMatrix)*transpose(GCP(2,5:6));
GCP(3,7:8)=transpose(RotationMatrix)*transpose(GCP(3,5:6));
LinearOffsets=[mean([GCP(1,7)-GCP(1,1),GCP(2,7)-GCP(2,1),GCP(3,7)-GCP(3,1)]);mean([GCP(1,8)-GCP(1,2),GCP(2,8)-GCP(2,2),GCP(3,8)-GCP(3,2)])];
GCP(1:3,9:10)=GCP(1:3,7:8)-transpose(LinearOffsets);
CalibrationErrors=[sqrt((GCP(1,1)-GCP(1,9))^2+(GCP(1,2)-GCP(1,10))^2),sqrt((GCP(2,1)-GCP(2,9))^2+(GCP(2,2)-GCP(2,10))^2),sqrt((GCP(3,1)-GCP(3,9))^2+(GCP(3,2)-GCP(3,10))^2)];
end