%%----------------------------------------------------------------
%                 Start Mask-RCNN via Command Window
%
%           Training / Detection Python Files are called here
%           Allows functionalities according to 
%           https://github.com/matterport/Mask_RCNN
%           --> Training, Resume Training, Detection, Evaluation
%
%                       Assumes Windows 10 as OS
%
%                               O O
%                              \___/
%----------------------------------------------------------------
% BSD 3-Clause License
%
% Copyright (c) 2020, Kruber F., Sanchez, E.
% All rights reserved.
%----------------------------------------------------------------

function [] = startMaskRCNN(runMode, thisDir, dataDirRoot, outputDirImages, dataDirTrain, weightsSource)
%This function handles the process to start Mask RCNN via command window,
%which is called through Matlab. 


tmpDir=dir([outputDirImages '/*.jpg']); %get number of Images
numImg=size(tmpDir,1);

outputDirImagesCut = outputDirImages;
outputDirImagesCut = strsplit(outputDirImagesCut,'\');
if strcmp(outputDirImagesCut{end},'registered')
outputDirImagesCut = [outputDirImagesCut{end-1},'\',outputDirImagesCut{end}];
else
  outputDirImagesCut = [outputDirImagesCut{end}];  
end

if strcmp(runMode, 'train_coco')
    % Train a new model starting from pre-trained COCO weights
    disp('Start Python and Training Mask-RCNN from COCO weights')
    disp('=====================================================')
    runPy = ['python vehDetection.py train --dataset=',dataDirTrain,' --weights=coco'];
elseif strcmp(runMode, 'train_resume')
    disp('Start Python and Training Mask-RCNN from last weights')
    disp('=====================================================')
    runPy = ['python vehDetection.py train --dataset=',dataDirTrain,' --weights=last'];
elseif strcmp(runMode, 'train_specifyWeights')
    disp(['Start Python and Training Mask-RCNN from specified weights:'])
    disp(['Source Location: ',weightsSource])
    disp('=====================================================')
    runPy = ['python vehDetection.py train --dataset=',dataDirTrain,' --weights=',weightsSource];
elseif strcmp(runMode, 'detect') 
    disp('Start Python and Mask-RCNN to detect vehicles')
    disp(['Note: Processing time is approx. 1 s to 2 s for Full HD images. In this case: ',num2str(numImg),' s to ', num2str(numImg*2), ' s for all images'])
    disp('=====================================================')
    if isempty(weightsSource)
        runPy = ['python vehDetection.py detect --dataset=',dataDirRoot,' --subset=',outputDirImagesCut,' --weights=last'];
    else
        runPy = ['python vehDetection.py detect --dataset=',dataDirRoot,' --subset=',outputDirImagesCut,' --weights=',weightsSource];
    end
end

%% Start Command Window
runFolder = ['cd ',thisDir];
system(runFolder);
system(runPy,'-echo');

end