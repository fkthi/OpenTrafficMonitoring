%%----------------------------------------------------------------
%                       Post Processing Main Script - Publication Example
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
addpath([pwd,'\vehicle_detection'])
addpath([pwd,'\post_processing'])

VideoExample=[29]; % [50m] example

whichWeights = 'generalWeights'; % Choose 'specializedWeights' or 'generalWeights'
compareToReference = 1; % if error plots should be created, s. below

global NumOfProcessType processType FirstProcessType
FirstProcessType=4;
NumOfProcessType=4;

for ChooseVideo= 1:length(VideoExample)
    for processType = FirstProcessType:1:NumOfProcessType
       
        VideoID = VideoExample(ChooseVideo);
        if processType==1
            ImgProcessType = 'raw';
            processTypeTitle=' Raw';
        end
        if processType==2
            ImgProcessType = 'reg';
            processTypeTitle=' Registered';
        end
        if processType==3  %Perform registration and Polygonshift (all corners radially to image center)
            ImgProcessType = 'reg';
            processTypeTitle=' Reg + Polygon shift';
        end
        if processType==4  %Perform registration and Polygonshift (all corners radially to image center)
            ImgProcessType = 'reg';
            processTypeTitle=' Reg + Polygon shift';
        end
        
        %%===============================================================
        %% Load Data for selected Video (DroneHeight, Resolution, GCPs etc.
        run loadVideoData_syncedTime.m
        %%===============================================================
        
        
        %%===============================================================
        %% Compute Offsets based on loaded Video (and VideoMeta data)
        %%===============================================================
        if DoComputeOffsets     %see loadVideoData_syncedTime.m
            [OrientationOffset, LinearOffsets, CalibrationErrors, GCP] = ComputeOffsets(meterToPx, GCP);
        else
            OrientationOffset = 0;
            LinearOffsets = 0;
        end
        
        %%===============================================================
        %% Init Variables
        %%===============================================================
        
        OutterHeight = 0.8;  %outer vehicle corner height estimation
        InnerHeight = 0.15;  %inner vehicle corner height estimation
        standStill = 0.3;  % [m/s] threshold -> below is treated as standStill (in KalmanFilter & CleanUp)
        fps = 50;
        minTrackTime = 30; %[seconds]: all object below minTrackTime will be cleaned
        invisibleForTooLong = 3; % [frames]: if object invisible for >= invisibleForTooLong --> track lost (used in Tracker + CleanUp)
        cleanUp = true;
        clearUncleanedVars = false; % see Cleanup Routine below --> should uncleaned variables be deleted?
        showVideo = true;
        recordMP4 = true; %records a mp4 file in folder saveVideoPath
        saveVideoPath = [pwd,'\_savedVideos'];
        %%===============================================================
        %% Run Kalman Filter + Tracker
        %%===============================================================
        
        [posX, posY, velX_GTP, velY_GTP, speed, accX_LTP, accY_LTP, acc_magn, trackIDs, CourseOG, yaw, veh_length, veh_width, CarCornersKF_tracker, CarCorners_tracker, yawInImg] = ...
            KFandTracker(mrcnnOutput,resolution,droneHeight,meterToPx, standStill, OrientationOffset, LinearOffsets, fps, invisibleForTooLong,OutterHeight,InnerHeight);
        
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
                        CarCornersKF_trackerplot{q,qq} = CarCornersKF_tracker{q,qq} ./meterToPx;
                        CarCornersKF_trackerplot{q,qq}(:,2) = resolution(2)-(CarCornersKF_trackerplot{q,qq}(:,2)); %flip y axis
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
                    accX_LTP acc_magn qq q record outputDirImages MaskRCNNOutputLink currentResultDir ...
                    CarCorners_trackerplot CarCorners_tracker
            end
        end
        %%===============================================================
        %% Run / Record a Video
        % IMPORTANT: plotting KF based CarCorners, ID, Speed at posX,posY
        % not posibble with REFERENCE measurements, because points are rotated!
        %%===============================================================
        if showVideo && exist(imageSource)~=0
            if cleanUp
                VideoTrackerResults(resolution, imageSource, saveVideoPath, recordMP4, fps, speedClean, trackIDsClean, CarCorners_px_trackerClean, mrcnnOutput)
            else
                VideoTrackerResults(resolution, imageSource, saveVideoPath, recordMP4, fps, speed, trackIDs, CarCorners_tracker, mrcnnOutput)
            end
        else
            warning('WARNING: IMAGE SOURCE IS NOT AVAILABLE. DOWNLOAD THE DATA SET FIRST AND COPY INTO MAIN FOLDER');
        end
        
        %%===============================================================
        %% Plots -- Cumulative Error plot from IV2020 Paper
        %%===============================================================
        if compareToReference
            limErrMeter = 1.2; %for Plot function: separate error in meter/pixel
            %%Generating the time axis for the drone in miliseconds
            TimeAxisDroneMillis=DroneTimeStartms:20:(20*(size(posXClean,1)-1))+DroneTimeStartms;
            %%Generating the time axis for the drone in seconds
            TimeAxisSeconds=TimeAxisDroneMillis/1000;
            
            %%Generating the time axis for the ADMA in miliseconds <- interpolating to
            %%get measurements every millisecond
            XTimeADMAinterp=Data1_INS_Time_Milliseconds(1):1:Data1_INS_Time_Milliseconds(end); %% time axis to interpolate
            XPosInterpADMA = interp1(Data1_INS_Time_Milliseconds,Data1_INS_Pos_Rel_Longitude_POI,XTimeADMAinterp); %% inteprolating X axis
            YPosInterpADMA = interp1(Data1_INS_Time_Milliseconds,Data1_INS_Pos_Rel_Latitude_POI1,XTimeADMAinterp); %% inteprolating Y axis
            
            %%Generating the time axis for the Drone in miliseconds <- interpolating to
            %%get measurements every milisecond
            
            % clean NaN (replace NaN with fillmissing)
            if cleanUp
                aa = 0.5 * (fillmissing(posX, 'previous') + fillmissing(posX, 'next'));
                bb = 0.5 * (fillmissing(posY, 'previous') + fillmissing(posY, 'next'));
                posX = aa;
                posY = bb;
            end
            % end clean NaN
            
            DroneTimeToInterp=TimeAxisDroneMillis(1):1:TimeAxisDroneMillis(end); %% time axis to interpolate
            XPosInterpDrone = interp1(TimeAxisDroneMillis,posX(:,EgoID),DroneTimeToInterp); %% inteprolating X axis
            YPosInterpDrone = interp1(TimeAxisDroneMillis,posY(:,EgoID),DroneTimeToInterp); %% inteprolating Y axis
            YawInImgInterpDrone = interp1(TimeAxisDroneMillis,yawInImg(:,EgoID),DroneTimeToInterp); %% inteprolating yaw
            
            % Generating a common time axis
            CommonTimeAxis=max([DroneTimeToInterp(1),XTimeADMAinterp(1)]):1:min([DroneTimeToInterp(end),XTimeADMAinterp(end)]);
            [~,IdxMinADMA] = min(abs((XTimeADMAinterp-CommonTimeAxis(1)))); %% geting the index of the first ADMA measurement for the comparison
            [~,IdxMaxADMA] = min(abs((XTimeADMAinterp-CommonTimeAxis(end)))); %% geting the index of the last ADMA measurement for the comparison
            [~,IdxMinDrone] = min(abs((DroneTimeToInterp-CommonTimeAxis(1)))); %% geting the index of the first Drone measurement for the comparison
            [~,IdxMaxDrone] = min(abs((DroneTimeToInterp-CommonTimeAxis(end)))); %% geting the index of the last Drone measurement for the comparison
            
            % Estimating the error: Euclidean distance between the ADMA and the Drone %measurement
            ErrorFromCenter=nan(IdxMaxDrone-IdxMinDrone,1);
            for ErrorCenterIdx=0:IdxMaxDrone-IdxMinDrone
                ErrorFromCenter(ErrorCenterIdx+1,1)=sqrt((XPosInterpDrone(ErrorCenterIdx+IdxMinDrone)-XPosInterpADMA(ErrorCenterIdx+IdxMinADMA+FrameOffset))^2+(YPosInterpDrone(ErrorCenterIdx+IdxMinDrone)-YPosInterpADMA(ErrorCenterIdx+IdxMinADMA+FrameOffset))^2);
            end
            
            [binvalues,blabla]=PositioningConfidence(ErrorFromCenter(:,1),500,VideoID);
            close (blabla)
            % Compare the position outputs between Drone and ADMA (reference)
            run CompareAndPlot.m
        end
    end
end
%%===============================================================
%% END
%%===============================================================