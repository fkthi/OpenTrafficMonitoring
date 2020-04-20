%%----------------------------------------------------------------
%     Perform Image Registration based on Matlab Functions
%     Code is written to Run for the GUI Version as well
%                               O O
%                              \___/
%----------------------------------------------------------------
% BSD 3-Clause License
%
% Copyright (c) 2020, Kruber F., Sanchez, E.
% All rights reserved.
%----------------------------------------------------------------

function saveDir = image_registration(workingDir,saveDir,originalFrame,outputQuality,h)
%To use the function from commandline, use in this format
%image_registration('#working direcory','#save directory',originalframe,outputquality)
% Working directory : Actual directory of images file
% Save directory : can take any of the following
%       1.'subfolder' - Creates a subfolder to save the registered images
%       2.Actual directory to save the images
%Originalframe : Numeric (keyframe or reference image)
%Outputquality : Numeric (specify the quality of the images to be saved)

if nargin==5
    workingDir = get(h.path_source,'string');
    saveDir = get(h.path_save,'string');
    originalFrame = str2num(get(h.keyframe,'string'));
    outputQuality = str2num(get(h.quality,'string'));
elseif nargin ==4
    check_Inputs(workingDir,saveDir,originalFrame,outputQuality)
    if strcmp(saveDir,'subfolder')
        saveDir = [workingDir,'\registered'];
    end
    if ~exist(saveDir,'dir')
            mkdir(saveDir)
    end
else
    error('Invalid number of Inputs')
end
%% Find Image Rotation and Scale Using Automated Feature Matching
% This example shows how to automatically align two images that differ by a
% rotation and a scale change. It closely parallels another example titled
% <matlab:showdemo('RotationFitgeotransExample') Find Image Rotation and Scale>.
% Instead of using a manual approach to register the two images, it
% utilizes feature-based techniques found in the Computer Vision System
% Toolbox(TM) to automate the registration process.
%
% In this example, you will use |detectSURFFeatures| and
% |vision.GeometricTransformEstimator| System object to recover rotation
% angle and scale factor of a distorted image. You will then transform the
% distorted image to recover the original image.

% Copyright 1993-2014 The MathWorks, Inc.

%% Step 0: Read in all jpgs in folder & init variables

imageNames = dir(fullfile(workingDir,'*.jpg'));
imageNames = {imageNames.name}';

scale_recovered = zeros(numel(originalFrame+1 : length(imageNames)),1);
theta_recovered = zeros(numel(originalFrame+1 : length(imageNames)),1);

%% Step 1: Read Key image
% Bring an image into the workspace.
original =  imread(fullfile(workingDir,imageNames{originalFrame}));
original_gray = rgb2gray(original);
%imshow(original);

%% Step 2: Read other frame

for i = originalFrame+1 : length(imageNames)
    
    distorted =  imread(fullfile(workingDir,imageNames{i}));
    distorted_gray = rgb2gray(distorted);
    %figure, imshow(distorted)
    
    %%
    % You can experiment by varying the scale and rotation of the input image.
    % However, note that there is a limit to the amount you can vary the scale
    % before the feature detector fails to find enough features.
    
    %% Step 3: Find Matching Features Between Images
    % Detect features in both images.
    ptsOriginal  = detectSURFFeatures(original_gray);
    ptsDistorted = detectSURFFeatures(distorted_gray);
    
    %%
    % Extract feature descriptors.
    [featuresOriginal,   validPtsOriginal]  = extractFeatures(original_gray,  ptsOriginal);
    [featuresDistorted, validPtsDistorted]  = extractFeatures(distorted_gray, ptsDistorted);
    
    %%
    % Match features by using their descriptors.
    indexPairs = matchFeatures(featuresOriginal, featuresDistorted);
    
    %%
    % Retrieve locations of corresponding points for each image.
    matchedOriginal  = validPtsOriginal(indexPairs(:,1));
    matchedDistorted = validPtsDistorted(indexPairs(:,2));
    
    %%
    % Show point matches. Notice the presence of outliers.
    % figure;
    % showMatchedFeatures(original,distorted,matchedOriginal,matchedDistorted);
    % title('Putatively matched points (including outliers)');
    
    %% Step 4: Estimate Transformation
    % Find a transformation corresponding to the matching point pairs using the
    % statistically robust M-estimator SAmple Consensus (MSAC) algorithm, which
    % is a variant of the RANSAC algorithm. It removes outliers while computing
    % the transformation matrix. You may see varying results of the
    % transformation computation because of the random sampling employed by the
    % MSAC algorithm.
    [tform, inlierDistorted, inlierOriginal] = estimateGeometricTransform(...
        matchedDistorted, matchedOriginal, 'similarity');
    
    %%
    % Display matching point pairs used in the computation of the
    % transformation matrix.
    % figure;
    % showMatchedFeatures(original,distorted, inlierOriginal, inlierDistorted);
    % title('Matching points (inliers only)');
    % legend('ptsOriginal','ptsDistorted');
    
    %% Step 5: Solve for Scale and Angle
    % Use the geometric transform, TFORM, to recover
    % the scale and angle. Since we computed the transformation from the
    % distorted to the original image, we need to compute its inverse to
    % recover the distortion.
    %
    %  Let sc = scale*cos(theta)
    %  Let ss = scale*sin(theta)
    %
    %  Then, Tinv = [sc -ss  0;
    %                ss  sc  0;
    %                tx  ty  1]
    %
    %  where tx and ty are x and y translations, respectively.
    %
    
    %%
    % Compute the inverse transformation matrix.
    Tinv  = tform.invert.T;
    
    ss = Tinv(2,1);
    sc = Tinv(1,1);
    scale_recovered(i) = sqrt(ss*ss + sc*sc);
    theta_recovered(i) = atan2(ss,sc)*180/pi;
    
    %%
    % The recovered values should match your scale and angle values selected in
    % *Step 2: Resize and Rotate the Image*.
    
    %% Step 6: Recover the Original Image
    % Recover the original image by transforming the distorted image.
    outputView = imref2d(size(original));
    recovered  = imwarp(distorted,tform,'OutputView',outputView);
    
    %%
    % Compare |recovered| to |original| by looking at them side-by-side in a montage.
    %figure, imshowpair(original,recovered,'montage')
    
    %% Step 7: Save images and scale/theta information
    filenameTmp = imageNames{i};
    filename = [filenameTmp(1:end-4),'_reg.jpg'];
    fullname = fullfile(saveDir,filename);
    imwrite(recovered,fullname, 'Quality',outputQuality);
    if i == originalFrame+1     %save also first image!
        filenameTmp = imageNames{originalFrame};
        filename = [filenameTmp(1:end-4),'_reg.jpg'];
        fullname = fullfile(saveDir,filename);
        imwrite(original,fullname, 'Quality',outputQuality);
    end
    if mod(i,round(length(imageNames)/5))==0
        if nargin==5
            set(h.update,'string',strcat('Processed...',string(i),' out of...',string(length(imageNames)))); % to update the status in GUI
            pause(0.001)
        else
            disp(strcat('Processed...',string(i),' out of...',string(length(imageNames))));
        end
    end
end

% % % %% save scale and orientation values
% % %     save([saveDir, '\recovered_scale_theta_',filename(1:8),'.mat'],'scale_recovered','theta_recovered');
% % %     if nargin==4
% % %         disp(strcat('Processed...',string(length(imageNames)),' out of...',string(length(imageNames))));
% % %     else
% % %         set(h.update,'string',strcat('Processed...',string(length(imageNames)),' out of...',string(length(imageNames))))
% % %     end
%%
% The |recovered| (right) image quality does not match the |original| (left)
% image because of the distortion and recovery process. In particular, the
% image shrinking causes loss of information. The artifacts around the edges are
% due to the limited accuracy of the transformation. If you were to detect
% more points in *Step 4: Find Matching Features Between Images*,
% the transformation would be more accurate. For example, we could have
% used a corner detector, |detectFASTFeatures|, to complement the SURF
% feature detector which finds blobs. Image content and image size also
% impact the number of detected features.

end


function check_Inputs(workingDir,saveDir,originalFrame,outputQuality)
    if ~exist(workingDir, 'dir')
        error('Input Images Directory is incorrect')
    end
    if ~strcmp(saveDir,'subfolder')
        if ~exist(saveDir,'dir')
            error('Images save Directory is incorrect')
        end
    end
    if ~isnumeric(originalFrame)
        error('Keyframe is incorrect')
    end
    if ~isnumeric(outputQuality)
        error('Output Quality is incorrect')
    end
end
