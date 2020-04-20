%%----------------------------------------------------------------
%             Load MetaFile and define variables for Benchmark
%                               O O
%                              \___/
%----------------------------------------------------------------
% BSD 3-Clause License
%
% Copyright (c) 2020, Kruber F., Sanchez, E.
% All rights reserved.
%----------------------------------------------------------------

folderMetaFile = [pwd,'\_positionData\'];
% Define index from MetaFile 
% (kept if/else in case further examples are included)
if VideoID == 29
    mIdx = 1;
else
    error('VideoID wrong');
end

DoComputeOffsets = true;
EgoID = 1;
FrameOffset = 0;

load(strcat(folderMetaFile,'VideoMeta_FHD_IV2020.mat'));

% Image Source for plotting results in Video
if processType==1
    imageSource = [pwd,'\_images\DJI_00',num2str(VideoID),'_img_1920_resized'];
else
    imageSource = [pwd,'\_images\DJI_00',num2str(VideoID),'_img_1920_resized\registered'];
end

% Define which output from Mask RCNN should be taken (depending on training
% weights and with or without image registration
switch ImgProcessType
    case 'raw'
        if strcmp('specializedWeights',whichWeights)
            mrcnnOutput = strcat(folderMetaFile,'_MRCNN_output_DJI_00',num2str(VideoID),'_img_1920_resized_specialized_weights.mat');
        else
            mrcnnOutput = strcat(folderMetaFile,'_MRCNN_output_DJI_00',num2str(VideoID),'_img_1920_resized_general_weights.mat');
        end
    case 'reg'
        if strcmp('specializedWeights',whichWeights)
            mrcnnOutput = strcat(folderMetaFile,'_MRCNN_output_DJI_00',num2str(VideoID),'_img_1920_resized_registered_specialized_weights.mat');
        else
            mrcnnOutput = strcat(folderMetaFile,'_MRCNN_output_DJI_00',num2str(VideoID),'_img_1920_resized_registered_general_weights.mat');
        end
end

% Load ADMA Data (Reference Sensor)
load(strcat(folderMetaFile,'DJI_00',num2str(VideoID),'.mat'))  

% Define some variables
droneHeight = VideoMeta(mIdx).DroneHeigth;
resolution = VideoMeta(mIdx).Resolution;
meterToPx = mean([VideoMeta(mIdx).GCP1_GCP2_MeterToPx, VideoMeta(mIdx).GCP1_GCP3_MeterToPx, VideoMeta(mIdx).GCP2_GCP3_MeterToPx]);
GCP(1,1:4)= [VideoMeta(mIdx).GCP1_north_xy_BS, VideoMeta(mIdx).GCP1_north_xy_px];
GCP(2,1:4)= [VideoMeta(mIdx).GCP2_south_xy_BS, VideoMeta(mIdx).GCP2_south_xy_px];
GCP(3,1:4)= [VideoMeta(mIdx).GCP3_east_control_xy_BS, VideoMeta(mIdx).GCP3_east_control_xy_px];
DroneTimeStartms=VideoMeta(mIdx).FirstRegisteredFrameTimeADMA;
DroneTimeEndms=DroneTimeStartms+(20*(VideoMeta(mIdx).NumFramesRegistered-1));