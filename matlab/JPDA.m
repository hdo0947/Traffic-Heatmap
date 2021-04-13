clc; clear all; close all;
% Set tracker
@initcvekf;
tracker = trackerJPDA('TrackLogic','Integrated' ,'AssignmentThreshold',100,...
    'ConfirmationThreshold', 0.8, ...
    'DeletionThreshold', 0.75);
% Specify the initial position and velocity
    % For simplicity sake, push to 2D, exclude z, and take the first two frames
    % to find the corresponding velocity => start tracking in scene 3
tp = theaterPlot('XLimits',[-50 200],'YLimits',[-400 30]);
% Plot to visualize tracks and detections
trackP = trackPlotter(tp,'DisplayName','Tracks','MarkerFaceColor','g','HistoryDepth',0);
detectionP = detectionPlotter(tp,'DisplayName','Detections','MarkerFaceColor','r');
% To obtain the position and velocity, create position and velocity
% selectors
positionSelector = [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 0 0]; % [x, y, 0]
velocitySelector = [0 1 0 0 0 0; 0 0 0 1 0 0; 0 0 0 0 0 0 ]; % [vx, vy, 0]

 load('demo_track_314frame.mat');
% Tracking portion
dt = 1;
hold on;
for time = 1:dt:314
   
    % Create detections of the two objects with noise.
%     clear detection
%     detection(1) = objectDetection(time,pos_est(:,1)+1*randn(3,1));
%     detection(2) = objectDetection(time,pos_est(:,2)+1*randn(3,1));
        % This section will be replaced by the matrix of centroid data from
        %the scene. 
        % The centroid matrix will have the x,y values of the centroids
    %[sorted,I] = sort(Ptcentroid{time});
    clear detection;
    for i = 1:size(Ptcentroid{time},1)
       detection(i) = objectDetection(time,Ptcentroid{time}(i,:));
    end
    veh_pos = lidar_vehicle{time}(1:2,4);
    % Step the tracker through time with the detections.
    scatter(veh_pos(1),veh_pos(2),'.');
    if size(Ptcentroid{time},1) ~= 0
        [confirmed,tentative,alltracks,info] = tracker(detection,time);
    
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
        if numel(confirmed)>0
            labels = arrayfun(@(x)num2str([x.TrackID]),confirmed,'UniformOutput',false);
            trackP.plotTrack(pos,vel,labels);
        end
        pause(0.1);
    end
        %     pause(0.5);
    %detectionP.plotDetection(meas',measCov);
%     drawnow;
%     % Display the cost and marginal probability of distribution every eight
%     % seconds.
%     if time>0 && mod(time,8) == 0
%         disp(['At time t = ' num2str(time) ' seconds,']);
%         disp('The cost of assignment was: ')
%         disp(info.CostMatrix);
%         disp(['Number of clusters: ' num2str(numel(info.Clusters))]);
%         if numel(info.Clusters) == 1
%             
%             disp('The two tracks were in the same cluster.')
%             disp('Marginal probabilities of association:')
%             disp(info.Clusters{1}.MarginalProbabilities)
%         end
%         disp('-----------------------------')
%     end
end
