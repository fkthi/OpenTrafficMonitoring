%%----------------------------------------------------------------
%      Clean up Variables due to short-term detections
%                               O O
%                              \___/
%----------------------------------------------------------------
% BSD 3-Clause License
%
% Copyright (c) 2020, Kruber F., Sanchez, E.
% All rights reserved.
%----------------------------------------------------------------

function  [trackIDsClean,posXClean,posYClean,speedClean, accX_LTPClean, accY_LTPClean,acc_magnClean,yawClean, CourseOGClean, veh_lengthClean,veh_widthClean,CarCornersKF_trackerClean,CarCornersKF_px_trackerClean,CarCorners_trackerClean,CarCorners_px_trackerClean] = ...
    startCleanupTracking_KFbbox(trackIDs, posX,posY, speed, accX_LTP, accY_LTP, acc_magn, yaw, CourseOG, veh_length ,veh_width, CarCornersKF_tracker, CarCornersKF_trackerplot, CarCorners_tracker, CarCorners_trackerplot, fps, minTrackTime, standStill, cleanLowSpeed, cleanNaN, invisibleForTooLong)


%%===============================================================
%% Clean short Tracks: all tracks below "minTrackTime" seconds
%%===============================================================
minTrackFrames = minTrackTime*fps;

totalNoZeros = zeros(size(trackIDs,2),1);
trackIDsClean = trackIDs;
posXClean = posX;
posYClean = posY;
speedClean = speed;
accX_LTPClean = accX_LTP;
accY_LTPClean = accY_LTP;
acc_magnClean = acc_magn;
yawClean = yaw;
CourseOGClean = CourseOG;
veh_lengthClean = veh_length;
veh_widthClean = veh_width;
CarCornersKF_trackerClean = CarCornersKF_tracker; %CarCorner corrected from KF
CarCornersKF_px_trackerClean = CarCornersKF_trackerplot;
CarCorners_trackerClean = CarCorners_tracker; %CarCorner as detected from Mask RCNN
CarCorners_px_trackerClean = CarCorners_trackerplot;


for i = 1 : size(trackIDs,2)
    totalNoZeros(i) = nnz(trackIDs(:,i));
end


trackIDsClean(:,totalNoZeros<minTrackFrames) = [];
posXClean(:,totalNoZeros<minTrackFrames) = [];
posYClean(:,totalNoZeros<minTrackFrames) = [];
speedClean(:,totalNoZeros<minTrackFrames) = [];
accX_LTPClean(:,totalNoZeros<minTrackFrames) = [];
accY_LTPClean(:,totalNoZeros<minTrackFrames) = [];
acc_magnClean(:,totalNoZeros<minTrackFrames) = [];
yawClean(:,totalNoZeros<minTrackFrames) = [];
CourseOGClean(:,totalNoZeros<minTrackFrames) = [];
veh_lengthClean(:,totalNoZeros<minTrackFrames) = [];
veh_widthClean(:,totalNoZeros<minTrackFrames) = [];
CarCornersKF_trackerClean(:,totalNoZeros<minTrackFrames) = [];
CarCornersKF_px_trackerClean(:,totalNoZeros<minTrackFrames) = [];
CarCorners_trackerClean(:,totalNoZeros<minTrackFrames) = [];
CarCorners_px_trackerClean(:,totalNoZeros<minTrackFrames) = [];


%set the ID properly after the cleaning
for i = 1 : size(trackIDsClean,2)
    %allNonZerosIndexes= find(trackIDsClean(:,i));
    trackIDsClean(:,i) = i;
end

%%===============================================================
%% Remove all Objects with mean speed < standStill
%%===============================================================

if cleanLowSpeed
    speedMean = speedClean;
    speedMean(isnan(speedMean)) = 0;
    speedMean = mean(speedMean);
    standStillIdx = speedMean < standStill;
    
    trackIDsClean(:,standStillIdx) = [];
    posXClean(:,standStillIdx) = [];
    posYClean(:,standStillIdx) = [];
    speedClean(:,standStillIdx) = [];
    accX_LTPClean(:,standStillIdx) = [];
    accY_LTPClean(:,standStillIdx) = [];
    acc_magnClean(:,standStillIdx) = [];
    yawClean(:,standStillIdx) = [];
    CourseOGClean(:,standStillIdx) = [];
    veh_lengthClean(:,standStillIdx) = [];
    veh_widthClean(:,standStillIdx) = [];
    CarCornersKF_trackerClean(:,standStillIdx) = [];
    CarCornersKF_px_trackerClean(:,standStillIdx) = [];
    CarCorners_trackerClean(:,standStillIdx) = [];
    CarCorners_px_trackerClean(:,standStillIdx) = [];
    
    
    %set the ID properly after the cleaning
    for i = 1 : size(trackIDsClean,2)
        %allNonZerosIndexes= find(trackIDsClean(:,i));
        trackIDsClean(:,i) = i;
    end
    
end

%%===============================================================
%% Replace NaN values with interpolation. If track lost, replace these NaNs with 0
%%===============================================================
if cleanNaN
    % START loop over the vehicles
    for i = 1:size(posXClean,2)
        % START loop over the time instances
        for j = 2:size(posXClean,1)
            if isnan(posXClean(j,i))
                % START loop over the next id time instances
                if j+invisibleForTooLong < size(posXClean,1)
                    for k = 1:invisibleForTooLong
                        if ~isnan(posXClean(j+k,i))
                            break
                        end
                    end
                else
                    invisibleForTooLong = size(posXClean,1)-j;
                    for k = 1:invisibleForTooLong
                        if ~isnan(posXClean(j+k,i))
                            break
                        end
                    end
                end
                % END loop over the next id time instances
                % Setting the required variables as 0/Performing interpolation
                if (k == invisibleForTooLong) || (j+k == size(posXClean,1))
                    posXClean(j:j+k,i)=0;
                    posYClean(j:j+k,i)=0;
                    speedClean(j:j+k,i)=0;
                    accX_LTPClean(j:j+k,i)=0;
                    accY_LTPClean(j:j+k,i)=0;
                    acc_magnClean(j:j+k,i) = 0;
                    yawClean(j:j+k,i)=0;
                    CourseOGClean(j:j+k,i) = 0;
                    veh_lengthClean(j:j+k,i)=0;
                    veh_widthClean(j:j+k,i)=0;
                else
                    posXClean(j-1:j+k,i)=linspace(posXClean(j-1,i),...
                        posXClean(j+k,i),k+2);
                    posYClean(j-1:j+k,i)=linspace(posYClean(j-1,i),...
                        posYClean(j+k,i),k+2);
                    speedClean(j-1:j+k,i)=linspace(speedClean(j-1,i),...
                        speedClean(j+k,i),k+2);
                    accX_LTPClean(j-1:j+k,i)=...
                        linspace(accX_LTPClean(j-1,i),...
                        accX_LTPClean(j+k,i),k+2);
                    accY_LTPClean(j-1:j+k,i)=...
                        linspace(accY_LTPClean(j-1,i),...
                        accY_LTPClean(j+k,i),k+2);
                    acc_magnClean(j-1:j+k,i)=...
                        linspace(acc_magnClean(j-1,i),...
                        acc_magnClean(j+k,i),k+2);
                    yawClean(j-1:j+k,i)=...
                        linspace(yawClean(j-1,i),...
                        yawClean(j+k,i),k+2);
                    CourseOGClean(j-1:j+k,i)=...
                        linspace(CourseOGClean(j-1,i),...
                        CourseOGClean(j+k,i),k+2);
                    veh_lengthClean(j-1:j+k,i)=...
                        linspace(veh_lengthClean(j-1,i),...
                        veh_lengthClean(j+k,i),k+2);
                    veh_widthClean(j-1:j+k,i)=...
                        linspace(veh_widthClean(j-1,i),...
                        veh_widthClean(j+k,i),k+2);
                end
            end
        end
        % END loop over the time instances
    end
    % END loop over the vehicles
end
