clc; clear all; close all;


FolderName = '0014';

% load true_pose generated from readOxts.m
load(sprintf('true_pose_%s.mat', FolderName))
% Number of frames
files = dir(sprintf('../data_set/%s/oxts/data/*.txt',FolderName));
% N is the total number of frames:
N = length(files);

PtCluster_oppodir = cell(N,1);
PtCluster_samedir = cell(N,1);
Ptcentroid_oppodir = cell(N,1);
Ptcentroid_samedir = cell(N,1);
lidar_vehicle = cell(N,1);
opposite_lane_points = cell(N,1);

for i = 1:N
    groundfilename = sprintf('pcd_data/%s/%010dground.pcd', FolderName, i-1);
    notgroundfilename = sprintf('pcd_data/%s/%010dnotground.pcd', FolderName, i-1);
    
    % Plot_cars_no_ground_aug will return clusters and also centroid of
    % clusters from both lanes
    [Cluster_oppodir, Cluster_samedir, centroid_oppodir, centroid_samedir] = cars_no_ground_aug(groundfilename, notgroundfilename,pose{i});
    
%     hold on
    path_x = pose{i}(1,4);
    path_y = pose{i}(2,4);
    path_z = pose{i}(3,4);
    
    opposite_lane_point_temp = [0,8,0,1].';
    opposite_lane_point = pose{i} * opposite_lane_point_temp;
    opposite_lane_points{i} = opposite_lane_point(1:3);
    
%     hold on
%     scatter3(path_x,path_y,path_z,20,'red')
%     scatter3(opposite_lane_point(1), opposite_lane_point(2), opposite_lane_point(3), 20, 'green')
    
    PtCluster_oppodir{i} = Cluster_oppodir;
    Ptcentroid_oppodir{i} = centroid_oppodir;
    PtCluster_samedir{i} = Cluster_samedir;
    Ptcentroid_samedir{i} = centroid_samedir;
    lidar_vehicle{i} = pose{i};
    
end
% xlabel('x [m]')
% ylabel('y [m]')
% zlabel('z [m]')
% axis equal
% title('Potential moving objects in all the frames')

%% Make data storage
lane_one = zeros(N,1);
lane_two = zeros(N,1);

%% opposite direction:
@initcvekf;
tracker = trackerJPDA('TrackLogic','Integrated' ,'AssignmentThreshold',35,...
    'ConfirmationThreshold', 0.85, ...
    'DeletionThreshold', 0.8);

% tp = theaterPlot('XLimits',[-50 200],'YLimits',[-400 30]);
% % Plot to visualize tracks and detections
% trackP = trackPlotter(tp,'DisplayName','Tracks','MarkerFaceColor','g','HistoryDepth',0);
% detectionP = detectionPlotter(tp,'DisplayName','Detections','MarkerFaceColor','r');
% To obtain the position and velocity, create position and velocity
% selectors
positionSelector = [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 0 0]; % [x, y, 0]
velocitySelector = [0 1 0 0 0 0; 0 0 0 1 0 0; 0 0 0 0 0 0 ]; % [vx, vy, 0]

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
    for i = 1:size(Ptcentroid_oppodir{time},1)
       detection(i) = objectDetection(time,Ptcentroid_oppodir{time}(i,:));
    end
    
    % pose and velocity of the LiDAR vehicle:
    if time == 1
        veh_prev = [0;0];
    else
        veh_prev = lidar_vehicle{time-1}(1:2,4);
    end
    veh_pos = lidar_vehicle{time}(1:2,4);
    veh_vel = veh_pos - veh_prev;
    
    
    % Step the tracker through time with the detections.
%     scatter(veh_pos(1),veh_pos(2),'r.');
    if size(Ptcentroid_oppodir{time},1) ~= 0
        [confirmed,tentative,alltracks,info] = tracker(detection,time);
    
%     % Extract position, covariance and label info
        [pos,cov] = getTrackPositions(confirmed,positionSelector);
    %     % Extract the velocity info
        vel = getTrackVelocities(confirmed,velocitySelector);
        confirmed2 = confirmed;
        counter = 0;
        for j = 1:size(vel,1)
            vel_dir = vel(j,1:2) * veh_vel;
            if norm(vel(j,1:2)) < 0.003 || norm(vel(j,1:2)) > 10 || vel_dir > 0
                if j-counter ~= 0
                    confirmed2(j-counter,:) = [];
                    counter = counter + 1;
    %                 tempid = confirmed(j).TrackID;
    %                 confirmed = confirmed(confirmed(:).TrackID ~= tempid);
                end
            end
        end
        confiemed = confirmed2;
        lane_one(time) = size(confirmed,1);
        meas = cat(2,detection.Measurement);
        measCov = cat(3,detection.MeasurementNoise);
    %     % Update the plot if there are any tracks.
        if numel(confirmed)>0
            labels = arrayfun(@(x)num2str([x.TrackID]),confirmed,'UniformOutput',false);
%             trackP.plotTrack(pos,vel,labels);
        end
        pause(0.1);
    end
end


%% Same direction
@initcvekf;
tracker = trackerJPDA('TrackLogic','Integrated' ,'AssignmentThreshold',30,...
    'ConfirmationThreshold', 0.85, ...
    'DeletionThreshold', 0.8);
% 
% tp = theaterPlot('XLimits',[-50 200],'YLimits',[-400 30]);
% % Plot to visualize tracks and detections
% trackP = trackPlotter(tp,'DisplayName','Tracks','MarkerFaceColor','g','HistoryDepth',0);
% detectionP = detectionPlotter(tp,'DisplayName','Detections','MarkerFaceColor','r');
% To obtain the position and velocity, create position and velocity
% selectors
% positionSelector = [1 0 0 0 0 0; 0 0 1 0 0 0; 0 0 0 0 0 0]; % [x, y, 0]
velocitySelector = [0 1 0 0 0 0; 0 0 0 1 0 0; 0 0 0 0 0 0 ]; % [vx, vy, 0]

dt = 1;
% hold on;
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
    for i = 1:size(Ptcentroid_samedir{time},1)
       detection(i) = objectDetection(time,Ptcentroid_samedir{time}(i,:));
    end
    
    % pose and velocity of the LiDAR vehicle:
    if time == 1
        veh_prev = [0;0];
    else
        veh_prev = lidar_vehicle{time-1}(1:2,4);
    end
    veh_pos = lidar_vehicle{time}(1:2,4);
    veh_vel = veh_pos - veh_prev;
    
    % Step the tracker through time with the detections.
%     scatter(veh_pos(1),veh_pos(2),'r.');
    if size(Ptcentroid_samedir{time},1) ~= 0
        [confirmed,tentative,alltracks,info] = tracker(detection,time);
    
%     % Extract position, covariance and label info
        [pos,cov] = getTrackPositions(confirmed,positionSelector);
    %     % Extract the velocity info
        vel = getTrackVelocities(confirmed,velocitySelector);
        confirmed2 = confirmed;
        counter = 0;
        for j = 1:size(vel,1)
            vel_dir = vel(j,1:2) * veh_vel;
            if norm(vel(j,1:2)) < 0.3 || norm(vel(j,1:2)) > 10 || vel_dir < 0
                if j-counter ~= 0
                    confirmed2(j-counter,:) = [];
                    counter = counter + 1;
    %                 tempid = confirmed(j).TrackID;
    %                 confirmed = confirmed(confirmed(:).TrackID ~= tempid);
                end
            end
        end
        confiemed = confirmed2;
        lane_two(time) = size(confirmed,1);
        meas = cat(2,detection.Measurement);
        measCov = cat(3,detection.MeasurementNoise);
    %     % Update the plot if there are any tracks.
        if numel(confirmed)>0
            labels = arrayfun(@(x)num2str([x.TrackID]),confirmed,'UniformOutput',false);
%             trackP.plotTrack(pos,vel,labels);
        end
        pause(0.1);
    end

end

%% Plotting for heatmap
val = 15;
x_l1 = zeros(N,1);
y_l1 = zeros(N,1);
x_l2 = zeros(N,1);
y_l2 = zeros(N,1);
% extract lane points from cell
for i = 1:N
    x_l1(i) = opposite_lane_points{i}(1);
    y_l1(i) = opposite_lane_points{i}(2);
    x_l2(i) = lidar_vehicle{i}(1,4);
    y_l2(i) = lidar_vehicle{i}(2,4);
end
l2_intensity = zeros(N, 1);
l1_intensity = zeros(N, 1);
for i = val+1:val:N
    % one moving in frame is yellow, zero is green more than 1 is red
    figure(200)
    l1_avg = sum(lane_one(i-val:i))/val;
    l2_avg = sum(lane_two(i-val:i))/val;
%     disp([x_l1(i-val:i) , y_l1(i-val:i)]);
    disp([x_l2(i-val:i) , y_l2(i-val:i)]);
    if l1_avg < 1
        scatter(x_l1(i-val:i) , y_l1(i-val:i),15,'g','filled')
        l1_intensity(i-val:i, 1) = 0;
    elseif l1_avg > 2
        scatter(x_l1(i-val:i) , y_l1(i-val:i),15,'r','filled')
        l1_intensity(i-val:i, 1) = 2;
    else
        scatter(x_l1(i-val:i) , y_l1(i-val:i),15,'y','filled')
        l1_intensity(i-val:i, 1) = 1;
    end
    if l2_avg < 1
        scatter(x_l2(i-val:i) , y_l2(i-val:i),15,'g','filled')
        l2_intensity(i-val:i, 1) = 0;
    elseif l2_avg > 2
        scatter(x_l2(i-val:i) , y_l2(i-val:i),15,'r','filled')
        l2_intensity(i-val:i, 1) = 2;
    else
        scatter(x_l2(i-val:i) , y_l2(i-val:i),15,'y','filled')
        l2_intensity(i-val:i, 1) = 1;
    end
    hold on
    axis equal
end
disp(l2_intensity);
disp(l1_intensity);
save("l1_intensity.mat", 'l1_intensity');
save("l2_intensity.mat", 'l2_intensity');
xlabel('x [m]')
ylabel('y [m]')
grid on
axis equal