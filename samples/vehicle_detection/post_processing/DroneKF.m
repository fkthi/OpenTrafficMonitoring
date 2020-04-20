%%----------------------------------------------------------------
%                        Kalman-Filter for Drone
%                               O O
%                              \___/
%----------------------------------------------------------------
% BSD 3-Clause License
%
% Copyright (c) 2020, Sanchez, E., Kruber F.
% All rights reserved.
%----------------------------------------------------------------
%Input: frames per second of video, image resolution, drone height, measurement vector for kalman filter, previous state vector, previous system covariance matrix, standStill, vehicle length, width, Corners of the bounding box
%Output: new state vector, system covariance matrix, extra state variables and corners of the bounding box
function [XNew,SystemCovarianceMatrix,ExtrasNew,CarCorners] = DroneKF(fps,Resolution,DroneHeight,Z,XOld,SystemCovarianceMatrix,standStill,l,w,CarCorners)
%COG: Course Over Ground
if isfinite(Z)%<- if the measurement vector IS valid
    %% Raw COG: slow driving
    if sqrt(XOld(4,1)^2+XOld(3,1)^2)>standStill
        RawCOG=atan2(XOld(4,1),XOld(3,1)); %<- If the vehicle is moving, estimate the COG from the velocity over ground.
    else
        RawCOG=XOld(7,1); %<- if the vehicle is not moving, recall the last COG
    end
    %% Correcting the vehicle orientation--> Approaching the Orientation to the COG:
    % The vehicle orientation should be close to the COG (driving forward only)
    if ((pi/2)<max([Z(3,1),RawCOG])-min([Z(3,1),RawCOG])) && ((pi*3/2)>max([Z(3,1),RawCOG])-min([Z(3,1),RawCOG])) % 90° to 270°
        if (Z(3,1)-RawCOG)>0
            Z(3,1)=Z(3,1)-pi;
        else
            Z(3,1)=Z(3,1)+pi;
        end
    elseif ((pi*3/2)<max([Z(3,1),RawCOG])-min([Z(3,1),RawCOG])) % 270° to 360°
        if (Z(3,1)-RawCOG)>0
            Z(3,1)=Z(3,1)-(2*pi);
        else
            Z(3,1)=Z(3,1)+(2*pi);
        end
    else  % 0 to 90°
        Z(3,1)=Z(3,1);
    end    
    H=[1,0,0,0,0,0,0,0;
        0,1,0,0,0,0,0,0;
        0,0,0,0,0,0,1,0];    
else%<- if the measurement vector is NOT valid, we predict the sate, but make no correction with sensor
    H=[0,0,0,0,0,0,0,0;
        0,0,0,0,0,0,0,0;
        0,0,0,0,0,0,0,0];
    Z = [999;999;999];  %dummy number, if no measurement (XNew = xpred + K * (Z - H*xpred);)
end

%% Tuning parameters: Measurement noise and system noise
if Resolution(1) == 3840 %<- 4K 
    MeasurementNoise=diag([0.074; 0.075;0.027]);
    SystemNoise=diag([0.3,0.3,.1,.1,3,3,1,1]);
elseif Resolution(1) == 1920 && DroneHeight <= 50 %<- FHD at 50 m
    MeasurementNoise=diag([0.0149; 0.0149;0.0583]);
    SystemNoise=diag([.05,.05,.1,.1,3,3,1,1]);
elseif Resolution(1) == 1920 && DroneHeight >50 && DroneHeight <= 75 %<- FHD at 75 m
    MeasurementNoise=diag([0.0838;0.0838;0.0843]);
    SystemNoise=diag([.104,.104,.1,.1,3,3,1,1]);
    elseif Resolution(1) == 1920 && DroneHeight >75 && DroneHeight <= 100 %<- FHD at 100 m
    MeasurementNoise=diag([0.0838;0.0838;0.0843]);
    SystemNoise=diag([.104,.104,.1,.1,3,3,1,1]);
else
    error('FK ERROR: Resolution / DroneHeight not Supported yet! see function DroneKF');
end

dt=1/fps; %<- time between measurements
% System model:
SystemModel=[1,0,dt,0,(dt^2)/2,0,0,0;
    0,1,0,dt,0,(dt^2)/2,0,0;
    0,0,1,0,dt,0,0,0;
    0,0,0,1,0,dt,0,0;
    0,0,0,0,1,0,0,0;
    0,0,0,0,0,1,0,0;
    0,0,0,0,0,0,1,dt;
    0,0,0,0,0,0,0,1];

% 1. Predicting the motion of the vehicle:
xpred=SystemModel*XOld;
% 2.- Covariance of the state estimation (PRIOR to the measurements):
Pn_n1 = (SystemModel * SystemCovarianceMatrix * SystemModel.' + SystemNoise);
% 3. Kalman Gain:
K = (Pn_n1 * H') / (H * Pn_n1 * H.' + MeasurementNoise);
% 4. Optimal state (prediction + correction):
XNew = xpred + K * (Z - H*xpred);
% 5. Updated system covariance (AFTER the correction step):
SystemCovarianceMatrix = ((eye(size(SystemModel,1)) - K * H) * Pn_n1);

%% Estimating vehicle state variables that are not part of the state vector:
if sqrt(XNew(3,1)^2+XNew(4,1)^2)<standStill
    ExtrasNew(3,1)=XOld(7,1); %<- Course over ground
else
    ExtrasNew(3,1)=atan2(XNew(4,1),XNew(3,1)); %<- Course over ground
end
ExtrasNew(1,1)=(XNew(3,1).^2+XNew(4,1).^2).^(1/2);%<- magnitude of velocity
ExtrasNew(2,1)=(XNew(5,1).^2+XNew(6,1).^2).^(1/2); %<- magnitude of acceleration
ExtrasNew(4,1)=ExtrasNew(3,1)-XNew(7,1); %<- Sideslip
ExtrasNew(5:6,1)=[cos(-XNew(7,1)),-sin(-XNew(7,1));sin(-XNew(7,1)) cos(-XNew(7,1))]*[XNew(5,1);XNew(6,1)]; %<- Calculating accelerations in vehicle frame

%% Calculating the corners of the bounding box out of:
% 1.- the estimated state of the Kalman Filter (position and orientation)
% 2.- The known width and length of the vehicle
ex1=cos(XNew(7,1));
ex2=sin(XNew(7,1));
ey1=-ex2;
ey2=ex1;
% Corner 1
CarCorners(1,1)=XNew(1,1)+l/2*ex1-0.5*w*ey1;
CarCorners(1,2)=XNew(2,1)+l/2*ex2-0.5*w*ey2;
% Corner 2
CarCorners(2,1)=XNew(1,1)+l/2*ex1+0.5*w*ey1;
CarCorners(2,2)=XNew(2,1)+l/2*ex2+0.5*w*ey2;
% Corner 3
CarCorners(3,1)=XNew(1,1)-l/2*ex1+0.5*w*ey1;
CarCorners(3,2)=XNew(2,1)-l/2*ex2+0.5*w*ey2;
% Corner 4
CarCorners(4,1)=XNew(1,1)-l/2*ex1-0.5*w*ey1;
CarCorners(4,2)=XNew(2,1)-l/2*ex2-0.5*w*ey2;
end