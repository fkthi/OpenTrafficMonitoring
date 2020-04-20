%%----------------------------------------------------------------
%           Compare to Reference - Show Error Plot
%                               O O
%                              \___/
%----------------------------------------------------------------
% BSD 3-Clause License
%
% Copyright (c) 2020, Kruber F., Sanchez, E.
% All rights reserved.
%----------------------------------------------------------------

n0=pwd; % for storing plots as *png


global NumOfProcessType FirstProcessType processType processTypeIdx 

%%Generating the time axis for the drone in miliseconds
TimeAxisDroneMillis=DroneTimeStartms:20:(20*(size(posXClean,1)-1))+DroneTimeStartms;
%%Generating the time axis for the drone in seconds
TimeAxisSeconds=TimeAxisDroneMillis/1000;

%%Generating the time axis for the ADMA in miliseconds <- interpolating to
%%get measurements every milisecond
XTimeADMAinterp=Data1_INS_Time_Milliseconds(1):1:Data1_INS_Time_Milliseconds(end); %% time axis to interpolate
XPosInterpADMA = interp1(Data1_INS_Time_Milliseconds,Data1_INS_Pos_Rel_Longitude_POI,XTimeADMAinterp); %% inteprolating X axis
YPosInterpADMA = interp1(Data1_INS_Time_Milliseconds,Data1_INS_Pos_Rel_Latitude_POI1,XTimeADMAinterp); %% inteprolating Y axis
%%Generating the time axis for the Drone in miliseconds <- interpolating to
%%get measurements every milisecond
DroneTimeToInterp=TimeAxisDroneMillis(1):1:TimeAxisDroneMillis(end); %% time axis to interpolate
XPosInterpDrone = interp1(TimeAxisDroneMillis,posX(:,EgoID),DroneTimeToInterp); %% inteprolating X axis
YPosInterpDrone = interp1(TimeAxisDroneMillis,posY(:,EgoID),DroneTimeToInterp); %% inteprolating Y axis
%
%%Generating a common time axis
CommonTimeAxis=max([DroneTimeToInterp(1),XTimeADMAinterp(1)]):1:min([DroneTimeToInterp(end),XTimeADMAinterp(end)]);
[~,IdxMinADMA] = min(abs((XTimeADMAinterp-CommonTimeAxis(1)))); %% geting the index of the first ADMA measurement for the comparison
[~,IdxMaxADMA] = min(abs((XTimeADMAinterp-CommonTimeAxis(end)))); %% geting the index of the last ADMA measurement for the comparison
[~,IdxMinDrone] = min(abs((DroneTimeToInterp-CommonTimeAxis(1)))); %% geting the index of the first Drone measurement for the comparison
[~,IdxMaxDrone] = min(abs((DroneTimeToInterp-CommonTimeAxis(end)))); %% geting the index of the last Drone measurement for the comparison

%%Estimating the error: Euclidean distance between the ADMA and the Drone
%%measurement
ErrorFromCenter=nan(IdxMaxDrone-IdxMinDrone,1);
for ErrorCenterIdx=0:IdxMaxDrone-IdxMinDrone
    ErrorFromCenter(ErrorCenterIdx+1,1)=sqrt((XPosInterpDrone(ErrorCenterIdx+IdxMinDrone)-XPosInterpADMA(ErrorCenterIdx+IdxMinADMA+FrameOffset))^2+(YPosInterpDrone(ErrorCenterIdx+IdxMinDrone)-YPosInterpADMA(ErrorCenterIdx+IdxMinADMA+FrameOffset))^2);
end

%Generating the plot of error vs percentage of measurements below error:
%(for the current test)
[binvalues,blabla]=PositioningConfidence(ErrorFromCenter(:,1),500,VideoID);
close (blabla)

%Generating the plot of error vs percentage of measurements below error:
%(for all measurements)
if processType==FirstProcessType
    DiagramM=figure('Name','Percentage of measurements smaller than abscissa','units','normalized','outerposition',[0 0 1 1]);
    plot(binvalues(:,1),binvalues(:,3)*100,'LineWidth',2,'DisplayName',num2str(processTypeTitle))
    hold on
    xlabel('Error (meters)')
    legend
    ylabel('%')
    grid on
    set(gca,'FontSize',20)
    FullTitle=strcat('Percentage of measurements smaller than abscissa',' for Video -',num2str(VideoExample(ChooseVideo)));
    title(FullTitle)
%     ylim([0 100])
%     xlim([0 0.25]) % if error in meters
    hold on;
else
    figure(DiagramM);
    plot(binvalues(:,1),binvalues(:,3)*100,'LineWidth',2,'DisplayName',num2str(processTypeTitle))
%     xlim([0 0.25]) % if error in meters
    hold on
end

%from here, in pixels: -->
%% Converting the error from meters to pixels:
ErrorFromCenter(:,1)=ErrorFromCenter(:,1)/meterToPx;

%Generating the plot of error vs percentage of measurements below error:
%(for the current test)
[binvalues,blabla]=PositioningConfidence(ErrorFromCenter(:,1),500,VideoID);
close (blabla)
if processType==FirstProcessType
    DiagramPix=figure('Name','Percentage of measurements smaller than abscissa','units','normalized','outerposition',[0 0 1 1]);
    plot(binvalues(:,1),binvalues(:,3)*100,'LineWidth',2,'DisplayName',num2str(processTypeTitle))
    hold on
    xlabel('Error (pixels)')
    legend
    ylabel('%')
    grid on
    set(gca,'FontSize',20)
    FullTitle=strcat('Percentage of measurements smaller than abscissa',' for Video -',num2str(VideoExample(ChooseVideo)));
    title(FullTitle)
%     ylim([0 100])
%     xlim([0 8])    % if error in pixels
    hold on
else
    figure(DiagramPix);
    plot(binvalues(:,1),binvalues(:,3)*100,'LineWidth',2,'DisplayName',num2str(processTypeTitle))
%     xlim([0 8])    % if error in pixels
    hold on
end

%% Save Plots
% If all test are performed, saved the plots
% if processType==NumOfProcessType
%     figure(DiagramPix);
%     
%     n2=num2str(VideoExample(ChooseVideo));
%     n3=' - processType  ';
%     n4=num2str(processType);
%     n6='.fig';
%     n7=' - Pixels';
%     filename=strcat(n0,n2,n3,n4,n7,n6);
%     savefig(DiagramPix,filename)
%     n6='.png';
%     n7='- Pixels';
%     filename=strcat(n0,n2,n3,n4,n7,n6);
%     saveas(gcf,filename)
%     close(DiagramPix);
%     
%     figure(DiagramM);
%     
%     n2=num2str(VideoExample(ChooseVideo));
%     n3=' - processType ';
%     n4=num2str(processType);
%     n6='.fig';
%     n7=' - Meter';
%     filename=strcat(n0,n2,n3,n4,n7,n6);
%     savefig(DiagramM,filename)
%     n6='.png';
%     n7='- Meter';
%     filename=strcat(n0,n2,n3,n4,n7,n6);
%     saveas(gcf,filename)
%     close(DiagramM);
%     
%     
%     DiagramWidth=figure('Name','Veh Width','units','normalized','outerposition',[0 0 1 1]);
%     figure;
%     plot(veh_widthClean)
%     n7 = '- width';
%     filename=strcat(n0,n2,n3,n4,n7,n6);
%     saveas(gcf,filename)
%     close(DiagramWidth);
%end