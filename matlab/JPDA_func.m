%% Initialization
% % Not part of the function. Need to create this before running loop over
% % different frames
% @initcvekf;
% tracker = trackerJPDA('TrackLogic','Integrated' ,'AssignmentThreshold',100,...
%     'ConfirmationThreshold', 0.9, ...
%     'DeletionThreshold', 0.85);
% 
% trackP = trackPlotter(tp,'DisplayName','Tracks','MarkerFaceColor','g','HistoryDepth',0);
% detectionP = detectionPlotter(tp,'DisplayName','Detections','MarkerFaceColor','r');
% tp = theaterPlot('XLimits',[-1 150],'YLimits',[-50 50]);
%% Function Section
% Inputs: time - frame number of the scene
%         ptcentroid - center coordinate of the centroids in world frame
% Output: number of objects tracked in the scene; the trackerJPDA will have confirmed, tentative and alltracks
% automatically updated. 
function [num_vehicles] = JPDA_func(time,tracker, confirmed,tentative,alltracks, Ptcentroid)
    
    
    % Add  all the detected centroids to the detection
    clear detection;
    for i = 1:size(Ptcentroid,1)
       detection(i) = objectDetection(time,Ptcentroid(i,:));
    end

    
    % Plotting Code
    % Selectors to extract the desired corrdiantes from the tracked positions    
%     positionSelector = [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 0 0]; % [x, y, 0]
%     velocitySelector = [0 1 0 0 0 0; 0 0 0 1 0 0; 0 0 0 0 0 0 ]; % [vx, vy, 0]
%         % Step the tracker through time with the detections.
     if size(Ptcentroid,1) ~= 0
        [confirmed,tentative,alltracks] = tracker(detection,time);
    
%     % Extract position, covariance and label info
        [pos,cov] = getTrackPositions(confirmed,positionSelector);
    %     % Extract the velocity info
        vel = getTrackVelocities(confirmed,velocitySelector);
        confirmed2 = confirmed;
        counter = 0;
        for j = 1:size(vel,1)
            if norm(vel(j,1:2)) < 0.1 || norm(vel(j,1:2)) > 10
                if j-counter ~= 0
                    confirmed2(j-counter,:) = [];
                    counter = counter + 1;
    %                 tempid = confirmed(j).TrackID;
    %                 confirmed = confirmed(confirmed(:).TrackID ~= tempid);
                end
            end
        end
        confiemed = confirmed2;
        meas = cat(2,detection.Measurement);
        measCov = cat(3,detection.MeasurementNoise);
    %     % Update the plot if there are any tracks.
%         if numel(confirmed)>0
%             labels = arrayfun(@(x)num2str([x.TrackID]),confirmed,'UniformOutput',false);
%             trackP.plotTrack(pos,vel,labels);
%         end
%         pause(0.1);
    end
    
    
    
    
    num_vehicles = size(confirmed,1);
end
