%%----------------------------------------------------------------
%         Visualize the Bounding Boxes as a Video Sequence
%                               O O
%                              \___/
%----------------------------------------------------------------
% BSD 3-Clause License
%
% Copyright (c) 2020, Kruber F., Sanchez, E.
% All rights reserved.
%----------------------------------------------------------------

function [] = VideoTrackerResults(resolution, imageSource, saveVideoPath, record, fps, speed, trackIDstore, CarCorners_tracker, mrcnnOutput)

imageNames = dir(fullfile(imageSource,'*.jpg'));
imageNames = {imageNames.name}';
velkmh = speed*3.6;
nFrames = size(speed,1);
nAllVeh = size(speed,2);
ResultsName = split(mrcnnOutput,"\");
ResultsName = ResultsName{end};
videoName = ResultsName(1:end-4);
ResultsName = strrep(ResultsName,'_','\_');  % to display underscore correctly

hFig = figure;
MatlabVersionTooOld = verLessThan('matlab', '9.4'); %check if Matlab Version >= 2018a
if ~MatlabVersionTooOld
    hFig.WindowState = 'maximized'; % Requires R2018a
end

ax = gca;
addToFrame = 10;  %make a white frame around video --> to see whats happening outside
ax.XLim = [0-addToFrame resolution(1)+addToFrame];
ax.YLim = [0-addToFrame resolution(2)+addToFrame];
axis([0-addToFrame resolution(1)+addToFrame 0-addToFrame resolution(2)+addToFrame])

videoStoreLink = [saveVideoPath,'\',videoName,'_',num2str(fps),'fps'];

if record == true
    myVideo = VideoWriter(videoStoreLink, 'MPEG-4');
    myVideo.FrameRate = fps;
    myVideo.Quality = 40;
    open(myVideo)
end


for ii = 1 : nFrames
    %p1 = []; p2 = []; p3 = []; p4 = []; px = []; py = [];
    %rotated from points of boxesRotCell
    nonEmptyCells = find(~cellfun(@isempty,CarCorners_tracker(ii,:)));
    nVeh = length(nonEmptyCells);
    p1 = zeros(nVeh,2); p2 = zeros(nVeh,2); p3 = zeros(nVeh,2); p4 = zeros(nVeh,2); px = zeros(nVeh,5); py = zeros(nVeh,5);
    centroidVeh = zeros(nVeh,2);
    for k = 1 : nVeh
        p1(k,:) = CarCorners_tracker{ii,nonEmptyCells(k)}(1,1:2);
        p2(k,:) = CarCorners_tracker{ii,nonEmptyCells(k)}(2,1:2);
        p3(k,:) = CarCorners_tracker{ii,nonEmptyCells(k)}(3,1:2);
        p4(k,:) = CarCorners_tracker{ii,nonEmptyCells(k)}(4,1:2);
        px(k,:) = [p1(k,1),p2(k,1),p3(k,1),p4(k,1),p1(k,1)];
        py(k,:) = [p1(k,2),p2(k,2),p3(k,2),p4(k,2),p1(k,2)];
        centroidVeh(k,:) = [(max(px(k,:))+min(px(k,:)))/2, (max(py(k,:))+min(py(k,:)))/2];
    end
    
    img = imread(fullfile(imageSource,imageNames{ii}));
    a = imagesc(img);
    hold on;
    xlim([0-addToFrame resolution(1)+addToFrame])
    ylim([0-addToFrame resolution(2)+addToFrame])
    
    ResultsNameText = text(10,resolution(2)-10,ResultsName,'FontSize',9,'Color','red','FontWeight', 'bold'); %show mask rcnn output file
    
    b = [];  c = [];  d = [];
    for k = 1 : nVeh
        b(k) =  plot(px(k,:),py(k,:),'r','Linewidth',1);
        hold on;
        
        str = num2str(trackIDstore(ii,nonEmptyCells(k)));
        str2 = [num2str(floor(velkmh(ii,nonEmptyCells(k)))),' km/h'];
        c(k) = text(centroidVeh(k,1),centroidVeh(k,2),str,'FontSize',20,'Color','red', 'FontWeight', 'bold');
        d(k) = text(centroidVeh(k,1)-25,centroidVeh(k,2)-25,str2,'FontSize',10,'Color','c','FontWeight', 'bold');
    end
    
    str3 = [num2str(ii),' / ',num2str(nFrames)]; %show current frame
    e = text(round(resolution(1))/10,resolution(2)-50,str3,'FontSize',22,'Color','red','FontWeight', 'bold');
    
    if record == true
        F = getframe(gcf);
        drawnow
        writeVideo(myVideo,F);
    end
    pause(0.0001);
    
    delete(a);
    delete(b);
    delete(c);
    delete(d);
    delete(e);
    
    % show 10% intervalls as progress
    if mod(ii,round(length(imageNames)/10)) == 0
        progess = (ii/length(imageNames))*100;
        disp(['Progess: ', num2str(round(progess,1)), ' %']);
    end
end
if record == true
    close(myVideo)
    disp(['Location of video file: ', videoStoreLink,'.mp4'])
    disp('=====================================================')
end
close(hFig)

end %end function
