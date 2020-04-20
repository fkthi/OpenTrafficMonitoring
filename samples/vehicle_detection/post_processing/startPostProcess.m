%%----------------------------------------------------------------
%                       Post Processing Main Script
%                   Loads Benchmark Data, Runs Tracking + KF
%                   Plots results and Visualizes on Images
%                               O O
%                              \___/
%----------------------------------------------------------------
% BSD 3-Clause License
%
% Copyright (c) 2020, Kruber F., Sanchez, E.
% All rights reserved.
%----------------------------------------------------------------

%check current Folder. Go to Main Folder
pattern = "post_processing";
TF = contains(pwd,pattern);
if TF == true
    cd ..
end
addpath([pwd,'\pre_processing'])
addpath([pwd,'\post_processing'])

global  processType
processType=1;  %needed for other functions (to keep compatibility with functions needed for Publication)

a = exist('imagesFolderLoop');
b = exist('videoLoop');

if a
    VideoList = imagesFolderLoop;
elseif b
    VideoList = videoLoop;
else
    error('No valid video or image sources found.')
end

%%===============================================================
%% Compute Offsets based on loaded Video (and VideoMeta data)
%%===============================================================

if strcmp(purpose,'general')
    DoComputeOffsets = 0;
end

if DoComputeOffsets     %see loadVideoData_syncedTime.m
    [OrientationOffset, LinearOffsets, CalibrationErrors, GCP] = ComputeOffsets(meterToPx, GCP);
else
    OrientationOffset = 0;
    LinearOffsets = 0;
end
%%===============================================================
%% Init Variables
%%===============================================================

OutterHeight = 0.0;  %outer vehicle corner height estimation
InnerHeight = 0.0;  %inner vehicle corner height estimation
standStill = 0.3;  % [m/s] threshold -> below is treated as standStill (in KalmanFilter & CleanUp)
fps = 50;
minTrackTime = 3; %[seconds]: all object below minTrackTime will be cleaned
invisibleForTooLong = 3; % [frames]: if object invisible for >= invisibleForTooLong --> track lost (used in Tracker + CleanUp)
cleanUp = true;
clearUncleanedVars = false; % see Cleanup Routine below --> should uncleaned variables be deleted?
showVideo = true;
recordMP4 = true; %records a mp4 file in folder saveVideoPath
saveVideoPath = [pwd,'\_savedVideos'];
if ~exist(saveVideoPath, 'dir')
    mkdir(saveVideoPath)
end
%%===============================================================
%% Run Kalman Filter + Tracker
%%===============================================================

[posX, posY, velX_GTP, velY_GTP, speed, accX_LTP, accY_LTP, acc_magn, trackIDs, CourseOG, yaw, veh_length, veh_width, CarCornersKF_tracker, CarCorners_tracker, yawInImg] = ...
    KFandTracker(MaskRCNNOutputLink,resolution(:,iLoop),droneHeight(iLoop),meterToPxLoop(iLoop), standStill, OrientationOffset, LinearOffsets, fps, invisibleForTooLong,OutterHeight,InnerHeight);

%%===============================================================
%% Run Cleanup after Tracking (short-time objects etc.)
%%===============================================================
cleanLowSpeed = true; %clean Objects with mean speed < standStill
cleanNaN = true;  %replace all NaNs by interpolation

if cleanUp
    CarCornersKF_trackerplot = CarCornersKF_tracker;
    CarCorners_trackerplot = CarCorners_tracker; % todo: CarCorners_tracker given in pixel, not meter
    for q = 1:size(CarCornersKF_tracker,1)
        for qq = 1:size(CarCornersKF_tracker,2)
            if ~isempty(CarCornersKF_trackerplot{q,qq})
                CarCornersKF_trackerplot{q,qq} = CarCornersKF_tracker{q,qq} ./meterToPxLoop(iLoop);
                CarCornersKF_trackerplot{q,qq}(:,2) = resolution(2,iLoop)-(CarCornersKF_trackerplot{q,qq}(:,2)); %flip y axis
                CarCorners_trackerplot{q,qq} = CarCorners_tracker{q,qq}; %CarCorners_tracker given in pixel, not meter
                CarCorners_trackerplot{q,qq}(:,2) = (CarCorners_trackerplot{q,qq}(:,2)); % do not flip y axis --> done already.
            end
        end
    end
    
    [trackIDsClean,posXClean,posYClean,speedClean,accX_LTPClean, accY_LTPClean,acc_magnClean, yawClean, CourseOGClean, veh_lengthClean,veh_widthClean,CarCornersKF_trackerClean,CarCornersKF_px_trackerClean,CarCorners_trackerClean,CarCorners_px_trackerClean] = ...
        startCleanupTracking_KFbbox(trackIDs, posX,posY, speed, accX_LTP, accY_LTP, acc_magn, yaw, CourseOG, veh_length ,veh_width, CarCornersKF_tracker, CarCornersKF_trackerplot, CarCorners_tracker, CarCorners_trackerplot, fps, minTrackTime, standStill, cleanLowSpeed, cleanNaN, invisibleForTooLong);
    
    if clearUncleanedVars
        clearvars velY_GTP velX_GTP yaw veh_width veh_length trackIDs speed posYplot ...
            posY posXplot posX CourseOG CarCornersKF_trackerplot CarCornersKF_tracker  accY_LTP ...
            accX_LTP acc_magn qq q record outputDirImages ...
            CarCorners_trackerplot CarCorners_tracker
    end
end
%%===============================================================
%% Run / Record a Video
% IMPORTANT: plotting KF based CarCorners, ID, Speed at posX,posY
% not posibble with REFERENCE measurements, because points are rotated!
%%===============================================================
if showVideo && exist(outputDirImages)~=0
    if cleanUp
        VideoTrackerResults(resolution, outputDirImages, saveVideoPath, recordMP4, fps, speedClean, trackIDsClean, CarCorners_px_trackerClean, MaskRCNNOutputLink)
    else
        VideoTrackerResults(resolution, outputDirImages, saveVideoPath, recordMP4, fps, speed, trackIDs, CarCorners_tracker, MaskRCNNOutputLink)
    end
else
    warning('WARNING: IMAGE SOURCE IS NOT AVAILABLE. DOWNLOAD THE DATA SET FIRST AND COPY INTO MAIN FOLDER');
end

%%===============================================================
%% END
%%===============================================================