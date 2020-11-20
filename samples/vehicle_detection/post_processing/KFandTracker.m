%%----------------------------------------------------------------
%             Tracker + KF Filter for Drone Traffic Videos
%              Handles ID assignment, runs Kalman-Filter
%                               O O
%                              \___/
%----------------------------------------------------------------
% BSD 3-Clause License
%
% Copyright (c) 2020, Kruber F., Sanchez, E.
% All rights reserved.
%----------------------------------------------------------------

function [posX, posY, velX_GTP, velY_GTP, speed, accX_LTP, accY_LTP, acc_magn, trackIDs, CourseOG, yaw, veh_length, veh_width, CarCornersKF_tracker, CarCorners_tracker, yawInImg] = ...
    KFandTracker(mrcnn_output, Resolution, DroneHeight, MeterToPx, standStill, OrientationOffset, LinearOffsets ,fps, invisibleForTooLong,OutterHeight,InnerHeight)

input = load(mrcnn_output);
boxesRotCell = input.boxesRotCell;
boxesRotCell(1) = []; % %delete first cell entry (fake cell from Python, to keep always same cell shape)
nFrames = size(boxesRotCell,2);
boxesRotCellCleaned = boxesRotCell;  % Cleaned used if 1 vehicle has multiple detections. See assignment function
dt=1/fps; %%time delta in seconds for prediction  %todo: replace dt with fps !
costOfNonAssignment = 0.499; % if cost 0..1 --> must be smaller 0.5|Higher cost -> higher probability to be assigned to a track
nObjects = 1;
nextId = 1; % ID of the next track
%init variables to store
tracks = initializeTracks();
posX = zeros(length(boxesRotCell),nObjects);
posY = zeros(length(boxesRotCell),nObjects);
velX_GTP = zeros(length(boxesRotCell),nObjects);
velY_GTP = zeros(length(boxesRotCell),nObjects);
speed = zeros(length(boxesRotCell),nObjects);
accX_LTP = zeros(length(boxesRotCell),nObjects);
accY_LTP = zeros(length(boxesRotCell),nObjects);
acc_magn = zeros(length(boxesRotCell),nObjects);
trackIDs = zeros(length(boxesRotCell),nObjects);
CourseOG = zeros(length(boxesRotCell),nObjects);
yaw = zeros(length(boxesRotCell),nObjects);
veh_length = zeros(length(boxesRotCell),nObjects);
veh_width = zeros(length(boxesRotCell),nObjects);
yawInImg = zeros(length(boxesRotCell),nObjects);
CarCornersKF_tracker = {};
CarCorners_tracker = {};

for i=1:nFrames
    nVeh = size(boxesRotCell{1,i},1);
   
     [~, CarAreaPredict, PolyShapePredict] = predictNewLocationsOfTracks(); 
    [assignments, unassignedTracks, unassignedDetections] = ...
        detectionToTrackAssignment();
    updateAssignedTracks();
    updateUnassignedTracks();
    deleteLostTracks();
    createNewTracks();
    saveObjectList();   
end

    function tracks = initializeTracks()
        % create an empty array of tracks
        tracks = struct(...
            'id', {}, ...
            'kalmanFilter', {}, ...
            'age', {}, ...
            'totalVisibleCount', {}, ...
            'consecutiveInvisibleCount', {});
    end

    function [centroidsPredict, CarAreaPredict, PolyShapePredict] = predictNewLocationsOfTracks()
        nTracks = length(tracks);
        % Prediction task is performed in "unassigned tracks" function. Here we just load last state, independant of having a measurement or just prediction with system model  
        if nTracks > 0
            centroidsPredict = zeros(nTracks,2);
            CarAreaPredict = zeros(nTracks,1);
        else
            centroidsPredict = double.empty(nTracks,0);
            CarAreaPredict = double.empty(nTracks,0);
            PolyShapePredict= double.empty(nTracks,0);
        end
        for iii = 1 : nTracks 
            CarAreaPredict(iii) = polyarea(tracks(iii).kalmanFilter.CarCorners(:,1),tracks(iii).kalmanFilter.CarCorners(:,2)); %todo: Do actual Prediction ! Right now it is the past value!
            PolyShapePredict(iii) = polyshape(tracks(iii).kalmanFilter.CarCorners(:,1),Resolution(2)-tracks(iii).kalmanFilter.CarCorners(:,2)); %todo: Do actual Prediction ! Right now it is the past value!
        end   
    end

    function [assignments, unassignedTracks, unassignedDetections] = ...
            detectionToTrackAssignment()
        
        nTracks = length(tracks);
        nDetections = nVeh;
        % Compute the cost of assigning each detection to each track.
        
        centroidsDetect =zeros(nDetections,2);
        CarAreaDetect = zeros(nDetections,1);
        PolyShapeDetect = polyshape;
        
        for iii = 1:nDetections
            if size(boxesRotCell{1,i},3)>1 %% if the drone saw a vehicle
                CarCorners=[boxesRotCell{1,i}(iii,1,1),boxesRotCell{1,i}(iii,1,2);boxesRotCell{1,i}(iii,2,1),boxesRotCell{1,i}(iii,2,2);boxesRotCell{1,i}(iii,3,1),boxesRotCell{1,i}(iii,3,2);boxesRotCell{1,i}(iii,4,1),boxesRotCell{1,i}(iii,4,2)];
                CarCorners(:,2)=Resolution(2)-CarCorners(:,2); %flip y axis
                
                CarAreaDetect(iii) = polyarea(CarCorners(:,1),CarCorners(:,2));
                PolyShapeDetect(iii) = polyshape(CarCorners(:,1),CarCorners(:,2));  %todo: intesect of actual vs prediction !
                
            else%% if the drone saw NO vehicle
                CarCorners=nan(4,2);
                msg('NaN CarCorners');
            end
            centroidsDetect(iii, 1) = mean([max(CarCorners(:,1)), min(CarCorners(:,1))]); %centroid X
            centroidsDetect(iii, 2) = mean([max(CarCorners(:,2)), min(CarCorners(:,2))]); %centroid Y
        end
        
        %% if one vehicle has multiple detections -> take union and generate new carcorners and centroid, delete duplication --->>
        % Remark: Multiple detections for 1 vehicle usually appear due to trucks in crossings. This is
        % not a problem with the provided Example, but it is useful for
        % general purpose (public road data sets). The code is time
        % consuming and needs some improvement if performance matters.
        toDelete = [];
        IoUdetect = eye(nDetections, nDetections);
        Intersdetect= eye(nDetections, nDetections);
        for w = 1:nDetections
            for ww =  1:nDetections
                if w == ww  %numerical: sometimes slightly below or above 1, so init as eye() and skip w == w
                    continue
                end
                Intersdetect(w,ww) = area(intersect(PolyShapeDetect(w),PolyShapeDetect(ww))); % get are of polyshape object (which is intersection)
                IoUdetect(w,ww) = Intersdetect(w,ww) / (CarAreaDetect(w)+CarAreaDetect(ww)-Intersdetect(w,ww));
            end
        end
        
        for w = 1 : size(IoUdetect,2)
            if sum(IoUdetect(:,w)) > 1
                idx = find(IoUdetect(:,w));
                polyTmp = polyshape;
                for ww = 1: length(idx)
                    polyTmp = [polyTmp;PolyShapeDetect(idx(ww))];
                end
                polyTmp(1) = []; %delete the empty shape, which comes from the init of polyTmp
                unionP = union(polyTmp);
                unionPVertices = unionP.Vertices';
                CarCorners = minBoundingBox(unionPVertices)';
                
                CarAreaDetect(idx(1)) = polyarea(CarCorners(:,1),CarCorners(:,2));
                PolyShapeDetect(idx(1)) = polyshape(CarCorners(:,1),CarCorners(:,2));
                
                centroidsDetect(idx(1), 1) = mean([max(CarCorners(:,1)), min(CarCorners(:,1))]); %centroid X
                centroidsDetect(idx(1), 2) = mean([max(CarCorners(:,2)), min(CarCorners(:,2))]); %centroid Y
                
                toDelete = [toDelete,idx(2:end)'];  
            end
        end
        
        if ~isempty(toDelete)
            toDelete = unique(toDelete);
            centroidsDetect(toDelete, :) = [];
            PolyShapeDetect(toDelete) = [];
            CarAreaDetect(toDelete) = [];
            nDetections = size(centroidsDetect,1);
            boxesRotCellCleaned{1,i}(toDelete,:,:) = [];
        end
        %% <<--- if one vehicle has multiple detection -> take union and generate new carcorners and centroid, delete duplication
        
        
        %% centroid based tracking --->>
        %         if sum(centroidsPredict)>0  %if track exists
        %             if ~isempty(centroidsDetect) %if vehicle detected
        %                 cost = double(pdist2(centroidsPredict, double(centroidsDetect)));
        %             else
        %                 cost = double.empty(nTracks,0); %necessary to keep proper matrix size for "assignDetectionsToTracks" function
        %             end
        %         end
        %% <<--- centroid based tracking
        
        IoU = zeros(nTracks, nDetections);
        Inters = zeros(nTracks, nDetections);
        for w = 1:size(PolyShapePredict,2)
            for ww =  1:size(PolyShapeDetect,2)
                if ~isempty(PolyShapeDetect(ww).Vertices)
                    Inters(w,ww) = area(intersect(PolyShapePredict(w),PolyShapeDetect(ww))); % get area of polyshape object (which is an intersection)
                    IoU(w,ww) = Inters(w,ww) / (CarAreaPredict(w)+CarAreaDetect(ww)-Inters(w,ww));
                end
            end
        end
        
        % Solve the assignment problem. Returns Indizes of assignments
        [assignments, unassignedTracks, unassignedDetections] = ...
            assignDetectionsToTracks(1-IoU, costOfNonAssignment);
    end

%% Update Assigned Tracks

    function updateAssignedTracks()
        numAssignedTracks = size(assignments, 1);
        for k = 1:numAssignedTracks
            trackIdx = assignments(k, 1);
            detectionIdx = assignments(k, 2);
            
            if size(boxesRotCellCleaned{1,i},3)>1 %% if the drone saw a vehicle
                tracks(trackIdx).kalmanFilter.CarCorners=[boxesRotCellCleaned{1,i}(detectionIdx,1,1),boxesRotCellCleaned{1,i}(detectionIdx,1,2);boxesRotCellCleaned{1,i}(detectionIdx,2,1),boxesRotCellCleaned{1,i}(detectionIdx,2,2); ...
                    boxesRotCellCleaned{1,i}(detectionIdx,3,1),boxesRotCellCleaned{1,i}(detectionIdx,3,2);boxesRotCellCleaned{1,i}(detectionIdx,4,1),boxesRotCellCleaned{1,i}(detectionIdx,4,2)];
            else % if the drone saw NO vehicle
                tracks(trackIdx).kalmanFilter.CarCorners=nan(4,2);
            end
            
            [l,w,h,m,Z, CarCornersLengthClass]=lengthclass(tracks(trackIdx).kalmanFilter.CarCorners,Resolution,MeterToPx,OrientationOffset,LinearOffsets,DroneHeight,OutterHeight,InnerHeight);
            [XNew,SystemCovarianceMatrix,ExtrasNew, CarCornersKF] = DroneKF(fps, Resolution,DroneHeight, Z,tracks(trackIdx).kalmanFilter.XOld,tracks(trackIdx).kalmanFilter.SystemCovarianceMatrix,standStill,l,w,CarCornersLengthClass);
            
            tracks(trackIdx).kalmanFilter.X(:,i)=XNew;
            tracks(trackIdx).kalmanFilter.XOld=XNew;
            tracks(trackIdx).kalmanFilter.E(:,i)=ExtrasNew;
            tracks(trackIdx).kalmanFilter.SystemCovarianceMatrix = SystemCovarianceMatrix;
            tracks(trackIdx).kalmanFilter.CarLength = l;
            tracks(trackIdx).kalmanFilter.CarWidth = w;
            tracks(trackIdx).kalmanFilter.CarHeight = h;
            tracks(trackIdx).kalmanFilter.CarMass = m;
            tracks(trackIdx).kalmanFilter.CarCornersKF = CarCornersKF;

            % Update track's age.
            tracks(trackIdx).age = tracks(trackIdx).age + 1;
            
            % Update visibility.
            tracks(trackIdx).totalVisibleCount = ...
                tracks(trackIdx).totalVisibleCount + 1;
            tracks(trackIdx).consecutiveInvisibleCount = 0;
        end
    end

%% Update Unassigned Tracks
% Mark each unassigned track as invisible, and increase its age by 1.
    function updateUnassignedTracks()
        for m = 1:length(unassignedTracks)
            ind = unassignedTracks(m);
            % Run Kalman Filter, but set Z = nan since no measurement
            % available --> Predict based on system model
            [XNew,SystemCovarianceMatrix,ExtrasNew, CarCornersKF] = DroneKF(fps, Resolution,DroneHeight, [nan;nan;nan],tracks(ind).kalmanFilter.XOld,tracks(ind).kalmanFilter.SystemCovarianceMatrix,standStill,tracks(ind).kalmanFilter.CarLength,tracks(ind).kalmanFilter.CarWidth,tracks(ind).kalmanFilter.CarCornersKF);
            [CarCornersInPicture]=BackToPixel(CarCornersKF,MeterToPx,OrientationOffset,LinearOffsets);

            tracks(ind).kalmanFilter.X(:,i)=XNew;
            tracks(ind).kalmanFilter.XOld=XNew;
            tracks(ind).kalmanFilter.E(:,i)=ExtrasNew;
            tracks(ind).kalmanFilter.SystemCovarianceMatrix = SystemCovarianceMatrix;
            tracks(ind).kalmanFilter.CarCornersKF = CarCornersKF;
            tracks(ind).kalmanFilter.CarCorners = CarCornersInPicture;
            
            
            tracks(ind).age = tracks(ind).age + 1;
            tracks(ind).consecutiveInvisibleCount = ...
                tracks(ind).consecutiveInvisibleCount + 1;
        end
    end

%% Delete Lost Tracks
% The |deleteLostTracks| function deletes tracks that have been invisible
% for too many consecutive frames. 

    function deleteLostTracks()
        if isempty(tracks)
            return;
        end
        
        % Find the indices of 'lost' tracks.
        lostInds =  [tracks(:).consecutiveInvisibleCount] >= invisibleForTooLong;
        % Delete lost tracks.
        tracks = tracks(~lostInds);
    end

%% Create New Tracks
% Create new tracks from unassigned detections. Assume that any unassigned
% detection is a start of a new track.

    function createNewTracks()
        for o = 1:size(unassignedDetections, 1) 
            
            kf.CarLength = 0;
            kf.CarWidth = 0;
            kf.CarHeight = 0;
            kf.CarMass = 0;
            %%initializing the state vector and system covariance matrix
            kf.XOld=zeros(8,1);
            kf.SystemCovarianceMatrix=eye(size(kf.XOld,1));
            kf.SystemCovarianceMatrix(7,7) = kf.SystemCovarianceMatrix(7,7)*0.0001; %high damp initial orientation
            kf.X=nan(8,size(boxesRotCell,2));
            kf.E=nan(6,size(boxesRotCell,2));
            kf.CarCorners = {};
            kf.CarCornersKF = {};
            if size(boxesRotCell{1,i},3)>1 %% if the drone saw a vehicle
                kf.CarCorners=[boxesRotCell{1,i}(unassignedDetections(o),1,1),boxesRotCell{1,i}(unassignedDetections(o),1,2);boxesRotCell{1,i}(unassignedDetections(o),2,1),boxesRotCell{1,i}(unassignedDetections(o),2,2);boxesRotCell{1,i}(unassignedDetections(o),3,1),boxesRotCell{1,i}(unassignedDetections(o),3,2);boxesRotCell{1,i}(unassignedDetections(o),4,1),boxesRotCell{1,i}(unassignedDetections(o),4,2)];
            else % if the drone saw NO vehicle
                kf.CarCorners=nan(4,2);
            end
            
            [l,w,h,m,Z, CarCornersLengthClass]=lengthclass(kf.CarCorners,Resolution,MeterToPx,OrientationOffset,LinearOffsets,DroneHeight,OutterHeight,InnerHeight);
            [XNew,SystemCovarianceMatrix,ExtrasNew, CarCornersKF] = DroneKF(fps, Resolution,DroneHeight, Z,kf.XOld,kf.SystemCovarianceMatrix,standStill,l,w,CarCornersLengthClass);
            
            kf.X(:,i)=XNew;
            kf.XOld=XNew;
            kf.E(:,i)=ExtrasNew;
            kf.SystemCovarianceMatrix = SystemCovarianceMatrix;
            kf.CarLength = l; 
            kf.CarWidth = w;
            kf.CarHeight = h;
            kf.CarMass = m;
            kf.CarCornersKF = CarCornersKF;
            % CarCorners from detection is done above  (kf.CarCorners)
            
            % Create a new track.
            newTrack = struct(...
                'id', nextId, ...
                'kalmanFilter', kf, ...
                'age', 1, ...
                'totalVisibleCount', 1, ...
                'consecutiveInvisibleCount', 0);
            
            % Add it to the array of tracks.
            tracks(end + 1) = newTrack;
            
            % Increment the next id.
            nextId = nextId + 1;
        end
    end


    function saveObjectList()
        if ~isempty(tracks)
            trackID(:) = [tracks.id];
            for m = 1 : length(trackID)
                posX(i,tracks(m).id) = tracks(m).kalmanFilter.X(1,i);
                posY(i,tracks(m).id) = tracks(m).kalmanFilter.X(2,i);
                velX_GTP(i,tracks(m).id) = tracks(m).kalmanFilter.X(3,i);
                velY_GTP(i,tracks(m).id) = tracks(m).kalmanFilter.X(4,i);
                speed(i,tracks(m).id) = ((velX_GTP(i,tracks(m).id))^2+(velY_GTP(i,tracks(m).id))^2)^(1/2);
                accX_LTP(i,tracks(m).id) = tracks(m).kalmanFilter.E(5,i);
                accY_LTP(i,tracks(m).id) = tracks(m).kalmanFilter.E(6,i);
                acc_magn(i,tracks(m).id) = ((accX_LTP(i,tracks(m).id))^2+(accY_LTP(i,tracks(m).id))^2)^(1/2);
                CourseOG(i,tracks(m).id) = tracks(m).kalmanFilter.E(3,i);
                yaw(i,tracks(m).id) = tracks(m).kalmanFilter.X(7,i);
                trackIDs(i,tracks(m).id) = tracks(m).id;
                veh_length(i,tracks(m).id) = tracks(m).kalmanFilter.CarLength;
                veh_width(i,tracks(m).id) = tracks(m).kalmanFilter.CarWidth;
                CarCornersKF_tracker{i,tracks(m).id} = tracks(m).kalmanFilter.CarCornersKF;
                CarCorners_tracker{i,tracks(m).id} = tracks(m).kalmanFilter.CarCorners;
                yawInImg(i,tracks(m).id) = tracks(m).kalmanFilter.X(7,i) - OrientationOffset;
            end
        end
    end

end
