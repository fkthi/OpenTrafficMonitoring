%%----------------------------------------------------------------
%             Extract JPEG Frames from Video Input
%      Code is written to Run for the GUI Version as well
%                               O O
%                              \___/
%----------------------------------------------------------------
% BSD 3-Clause License
%
% Copyright (c) 2020, Kruber F., Sanchez, E.
% All rights reserved.
%----------------------------------------------------------------

function workingDir = images_From_Video(sourcefile,workingDir,calibration,stride,quality,resolution,h)
%   to call from command line, call with 6 arguments (sourcefile,workingDir,calibration,stride,quality,resolution)
%   sourcefile is a path of a video
%   workingDir can be any of the following
%       1. 'subfolder'  -   creates a folder in the videopath
%       2. Actual path to save the images
%   calibration can be any of the following
%       1. 'calibrate'  -   To calibrate, calibration files available for FHD and 4K
%       2. 'donotcalibrate' -   Not to calibrate
%       3. Actual path to a .mat calibration file, if incase user needs to use a user defined calibration file
%   stride - Numeric (for how often number of frames we need to create a image)
%   quality - Numeric (Output image quality)
%   resolution - Output resolution (can take any of the following)
%       1. 'FHD' - for 1920 X 1080 resolution
%       2. '4K' - for 3840 X 2160 resolution
%       3. '4Kc' - for 4096 X 2160 resolution
%       4. [width height] eg. [2000 1200] - for custom resolution
%       5. 'original' - for no change in resolution


gotcalibrationfileflag = 0;
calibrateflag = 0;
resizeflag = 0;

if nargin == 6
    %% If called via command window
    check_inputs(sourcefile,workingDir,calibration,stride,quality,resolution);
	[filepath,videoName,~] = fileparts(sourcefile);
    videoName2 = regexprep(videoName, ' +', '_'); %replace all spaces with underscore
    if ~strcmp(resolution,'original')
        resizeflag = 1;
        if strcmp(resolution,'FHD')
            width = 1920;
            height = 1080;
        elseif strcmp(resolution,'4K')
            width = 3840;
            height = 2160;
        elseif strcmp(resolution,'4Kc')
            width = 4096;
            height = 2160;
        else
            width = resolution(1,1);
            height = resolution(1,2);
        end
    else
        inputVideo = VideoReader(sourcefile);    
        width = inputVideo.width;
    end
    
    if strcmp(calibration,'calibrate')
        calibrateflag = 1;
    elseif ~strcmp(calibration,'donotcalibrate')
        load(calibration);
        temp = fields(load(calibration));
        cameraParams = eval(temp{1});
        gotcalibrationfileflag = 1;
    end
    
    if strcmp(workingDir,'subfolder')
        if ~strcmp(resolution,'original')
            if calibrateflag == 1
                workingDir = [filepath,'\',videoName2,'_img_',num2str(width),'_calib_resized'];
            else
            workingDir = [filepath,'\',videoName2,'_img_',num2str(width),'_resized'];
            end
        else
            if calibrateflag == 1
                workingDir = [filepath,'\',videoName2,'_img_calib_',num2str(width)];
            else
            workingDir = [filepath,'\',videoName2,'_img_',num2str(width)];
            end
        end
    end
    
elseif nargin==7
    %% If called via GUI
    [~,videoName,~] = fileparts(get(h.path_source,'string'));
    videoName2 = regexprep(videoName, ' +', '_'); %replace all spaces with underscore
    sourcefile = get(h.path_source,'string');
    workingDir = get(h.path_save,'string');
    stride = str2num(get(h.stride,'string'));
    calibrateflag = get(h.calibrate,'value');
    if calibrateflag && get(h.calibrate_other,'value')
        loadpath=get(h.calibrate_file,'string');
        load(loadpath);
        temp = fields(load(loadpath));
        cameraParams = eval(temp{1});
        gotcalibrationfileflag = 1;
    end
    resizeflag = 0;
    if get(h.resolution(2), 'value')
       resizeflag=1;
       width = 1920;
       height = 1080;
    end
    if get(h.resolution(3), 'value')
       resizeflag=1;
       width = 3840;
       height = 2160;
    end
    if get(h.resolution(4), 'value')
       resizeflag=1;
       width = 4096;
       height = 2160;
    end
    if get(h.resolution(5), 'value')
       resizeflag=1;
       width = str2num(get(h.manual_width,'string'));
       height = str2num(get(h.manual_height,'string'));
    end
    quality = str2num(get(h.quality,'string'));
else
      error('Invalid number of arguments');
end

if ~exist(workingDir,'dir')
    mkdir(workingDir);
end
inputVideo = VideoReader(sourcefile);
nFrames = floor(inputVideo.Frame * inputVideo.Duration);
nDigits = numel(num2str(nFrames));
ii = 1;

if calibrateflag && ~gotcalibrationfileflag
        error('SPECIFY THE CAMERA PARAMETER MATLAB FILE AND LOAD IT');
        %load cameraParams.mat;
end

if resizeflag == 1
    if width == inputVideo.Width && height == inputVideo.Height 
        resizeflag=0;
    end
end

while hasFrame(inputVideo)
    %% Actual process   
    img = readFrame(inputVideo);
    if mod(ii-1,stride) == 0
        subfilename = [videoName2,'_' sprintf(['%0',num2str(nDigits),'d'],ii)];
        if calibrateflag
            subfilename = strcat(subfilename,'_calib');
            img = undistortImage(img,cameraParams);
        end
        if resizeflag
            subfilename = strcat(subfilename,'_resized');
            img = imresize(img, [height width]);
        end
        filename = strcat(subfilename,'.jpg');
        fullname = fullfile(workingDir,filename);
        imwrite(img,fullname,'Quality',quality);
    end
    if mod(ii,round(nFrames/5))==0
        if nargin==7 
            set(h.update,'string',strcat('Processed...',string(ii),' out of...',string(nFrames),' Frames'));
        else
            disp(strcat('Processed...',string(ii),' out of...',string(nFrames),' Frames'));
        end
    end
    ii = ii+1;
end
    if nargin==7
        set(h.update,'string',strcat('Processed...',string(nFrames),' out of...',string(nFrames),' Frames'));
    else
        disp(strcat('Processed...',string(nFrames),' out of...',string(nFrames),' Frames'));
    end
end


function check_inputs(sourcefile,workingDir,calibration,stride,quality,sizein)
%% function to check whether the inputs are valid
if exist(sourcefile, 'file')
    if ~(strcmp(sourcefile(end-2:end),'mp4') || strcmp(sourcefile(end-2:end),'avi') || strcmp(sourcefile(end-2:end),'mov') || strcmp(sourcefile(end-2:end),'AVI') || strcmp(sourcefile(end-2:end),'MOV') || strcmp(sourcefile(end-2:end),'MP4')) 
        error('Sourcefile is incorrect');
    end
else
    error('Sourcefile is incorrect');
end
if ~(strcmp(workingDir,'subfolder'))
    if ~exist(workingDir, 'dir')
        error('Savepath is incorrect')
    end
end
if ~(strcmp(calibration,'calibrate') || strcmp(calibration, 'donotcalibrate'))
    if exist(calibration, 'file')
       if ~(strcmp(calibration(end-2:end),'mat'))
           error('Calibration is incorrect')
       end
    else
        error('Calibration is incorrect');
    end
else
    
end
if ~isnumeric(stride)
    error('Stride is incorrect');
end
if ~isnumeric(quality)
    error('Output Quality is incorrect');
end
if ~(strcmp(sizein,'FHD') || strcmp(sizein,'4K') || strcmp(sizein,'4Kc') || strcmp(sizein,'original'))
    if ~(size(sizein,1)==1 && size(sizein,2)==2 && isnumeric(size(1,1)) && isnumeric(size(1,2)))
        error('Output resolution is incorrect')
    end
end

end