%%----------------------------------------------------------------
%    After Processing Data - the Images can be merged to a Video
%                               O O
%                              \___/
%----------------------------------------------------------------
% BSD 3-Clause License
%
% Copyright (c) 2020, Kruber F., Sanchez, E.
% All rights reserved.
%----------------------------------------------------------------

function images_To_Video(workingDir,saveDir,Framerate,h)
% to call from commanline, call wih three arguments (Imagesdirectory,Savedirectory,famerate)
% Images directory : Actual path of the images
% Savedirectory : can take any of the following
%       1. 'samefolder' - to create in the same folder
%       2. Actual path to store the video.
%Framerate : Numeric(to set the frame rate of the video)

if nargin == 4
    workingDir = get(h.path_source,'string');
    saveDir = get(h.path_save,'string');
    Framerate = str2num(get(h.Framerate,'String'));
elseif nargin == 3
    check_Inputs(workingDir,saveDir,Framerate)
    if strcmp(saveDir,'samefolder')
        saveDir = workingDir;
    end
end

imageNames = dir(fullfile(workingDir,'*.jpg'));
imageNames = {imageNames.name}';

outputVideo = VideoWriter(fullfile(saveDir,'Final'), 'MPEG-4');
outputVideo.FrameRate = Framerate;
open(outputVideo)

for ii = 1:length(imageNames)
   img = imread(fullfile(workingDir,imageNames{ii}));
   writeVideo(outputVideo,img)
   if mod(ii,round(length(imageNames)/20))==0
       if nargin==4
           set(h.update,'string',strcat('Processed...',string(ii),' out of...',string(length(imageNames)),' images'));
           pause(0.001);
       else
           disp(strcat('Processed...',string(ii),' out of...',string(length(imageNames)),' images'));
       end
   end
end
if nargin==4
    set(h.update,'string',strcat('Processed...',string(length(imageNames)),' out of...',string(length(imageNames)),' images'));
else
    disp(strcat('Processed...',string(length(imageNames)),' out of...',string(length(imageNames)),' images'));
close(outputVideo)
end
end

function check_Inputs(workingDir,saveDir,Framerate)
if ~exist(workingDir,'dir')
    error('Input images folder is incorrect')
end
if ~exist(saveDir,'dir')
    if ~strcmp(saveDir,'samefolder')
        error('Save Directory is invalid')
    end
end
if ~isnumeric(Framerate)
	error('Framerate is invalid')
end
end