%%----------------------------------------------------------------
%                    Main Example File
%
%           This file calls all procedures from 
%           Pre-Processing to Detection to Post-Processing
%
%           Please Define Training Weights for Detection
%           Please Define Input Data (Video / Images)
%           Please Define if Registration should be performed
%
%                               O O
%                              \___/
%----------------------------------------------------------------
% BSD 3-Clause License
%
% Copyright (c) 2020, Kruber F., Sanchez, E.
% All rights reserved.
%----------------------------------------------------------------

clc;clear;

%% Add Subfolders to Matlab Path
addpath([pwd,'\pre_processing'])
addpath([pwd,'\post_processing'])

%% Define if Paper Specific process or general (paper specific: load offsets etc.) --> starts diefferent PostProcess script
purpose = 'general';  % 'paper', 'general'

%% LOAD MASK RCNN WEIGHTS:
%===================================================================

%if empty: take last weights from training - if a training was performed, else give complete link
rootDirectory = strsplit(pwd,'\'); rootDirectory = strjoin(rootDirectory(1:end-2),'\');
weightsSource =  [rootDirectory,'\logs\mask_rcnn_car_0400_791imgs_200312.h5'];  % download the provided logfile in the logs directory

%% Define variables
%===================================================================

doRegister = true; % Run Image Registration: true / false

% Example in case the input are videos --> generate jpgs first
videoLoop = {'DJI_0029_example.mp4'};
% Example in case the input are images -->
%imagesFolderLoop = {'DJI_0029_example_img_1920'};
meterToPxLoop = [0.0351];
originalRegisterFrameLoop = [1];
droneHeight = [50];
resolution = repmat([1920;1080],1,length(videoLoop));
runMode = 'detect'; % 'train_resume' 'detect' 'train_coco' 'train_specifyWeights'
sourceType = 'video'; % 'images' or 'video' (video only for detection)

%% Start Process
%===================================================================

if strcmp(sourceType, 'video')
    bigLoop = length(videoLoop);
else
    bigLoop = length(imagesFolderLoop);
end
if (bigLoop ~= length(originalRegisterFrameLoop)) || (bigLoop ~= length(meterToPxLoop))
    error('Error: check size of originalRegisterFrameLoop');
end


for iLoop = 1:bigLoop
    clearvars -except droneHeight resolution purpose weightsSource runMode sourceType bigLoop iLoop originalRegisterFrameLoop meterToPxLoop imagesFolderLoop videoLoop doRegister
    
    %% Define Folders: if Project structure is not changed, this code should be able to find all relvant folders in Windows.
    [thisDir, MaskRCNNRootDir, dataDirRoot, dataDirTrain, resultsDir] = startFolderHandling();
    % Note: if you want to specify weights for inference or resume training, adjust the variable "weightsSource"
    
    %% Define: Training / Inference (and pre-processing steps if inference)
    
    originalRegisterFrame = originalRegisterFrameLoop(iLoop); %which frame should be the key frame, to  which all other frames get aligned (orientation, translation,scale). Default: 1
    
    %% Set source files: video or images
    % If video is source, type "help images_From_Video" for more information
    if strcmp(sourceType, 'video')
        sourceFileVideo = videoLoop{iLoop}; 
        sourceFileVideoLink = [dataDirRoot,sourceFileVideo];
        strideVideo = 1; % every x-th videoFrame is converted to a JPG. Default: 1 (Every video frame)
        resizeVideo = 'original'; %'e.g. '4K' 'FHD' 'original'
    else
        imagesFolder = imagesFolderLoop{iLoop};
        outputDirImages = [dataDirRoot,imagesFolder];
    end
    
    %===================================================================
    %% Run Pre-Processing of Video
    outputImageQuality = 100; % integer 0...100
    if strcmp(sourceType, 'video')
            disp('Start storing but not calibrating images...')
            outputDirImages =images_From_Video(sourceFileVideoLink,'subfolder','donotcalibrate',strideVideo,outputImageQuality,resizeVideo);
        dataDir = outputDirImages;
        disp(['Storing images from video done.'])
        disp([' Location: ', outputDirImages])
        disp('=====================================================')
    end
    
    % Runs the image registration: creates subfolder and stores images. The only
    % Matlab Output variable is the new subfolder with the images.
    % if overrides the the variable outputDirImages from the "if strcmp(sourceType, 'video')" statement
    if doRegister
        disp('Start registering images and store them in subfolder...')
        if strcmp(sourceType, 'video')
            outputDirImages = image_registration(outputDirImages,'subfolder',originalRegisterFrame,outputImageQuality);
        else
            outputDirImages = image_registration(outputDirImages,'subfolder',originalRegisterFrame,outputImageQuality);
        end
        disp(['Registering and storing Images Done.'])
        disp([' Location: ', outputDirImages])
        disp('=====================================================')
    end
    disp('Image Pre-Processing Done.')
    disp('=====================================================')
    
    %===================================================================
    %% Run Training or Inference (Detection)
    disp('Start Mask-RCNN')
    disp('=====================================================')
    startMaskRCNN(runMode, thisDir, dataDirRoot, outputDirImages, dataDirTrain, weightsSource)
    %===================================================================
    %% Run Position estimation
    %Get the link of the Mask-RCNN Output
    if strcmp(runMode, 'detect')
        disp('=====================================================')
        disp('Start Tracking Process: Evaluate IDs, Positions, Speed etc.')
        ListResultsFolder = dir(resultsDir);
        currentResultDir = [ListResultsFolder(end).folder,'\' ListResultsFolder(end).name];
        MatFiles=dir(fullfile(currentResultDir,'*.mat'));
        MaskRCNNOutputLink = [currentResultDir,'\',MatFiles.name];  
        
        %% Perform the Multi Object Tracking (MOT)
        if strcmp(purpose,'general')
        run startPostProcess.m
        elseif strcmp(purpose,'paper')
            un startPostProcessPaperExample.m
        else
            error('Choose correct string for purpose: ''general'' or ''paper'' ')
        end
        disp('END OF POST-PROCESSING.')
        disp('=====================================================')
    end
end