%%----------------------------------------------------------------
%                    Folder Handling
%
%           It is assumed, that Mask-RCNN is prepared according to 
%           https://github.com/matterport/Mask_RCNN
%           Assumed structure: C\Mask-RCNN\...
%
%     Works for Windows. If Linux is used, change the delimiters etc.
%
%                               O O
%                              \___/
%----------------------------------------------------------------
% BSD 3-Clause License
%
% Copyright (c) 2020, Kruber F., Sanchez, E.
% All rights reserved.
%----------------------------------------------------------------

function [thisDir, MaskRCNNRootDir, dataDirRoot, dataDirTrain, resultsDir] = startFolderHandling()
%This functions gets current folder and sets the relevant folder names.
%Works for Windows. If Linux is used, change the delimiters etc.
%accordingly.

thisDir =  pwd;
tmpDirSplit = split(thisDir,'samples');
MaskRCNNRootDir = tmpDirSplit{1};
dataDirRoot = [MaskRCNNRootDir,'datasets\vehicle_detection\'];
dataDirTrain = [dataDirRoot,'__trainData\'];
resultsDir = [MaskRCNNRootDir,'results\'];

end

