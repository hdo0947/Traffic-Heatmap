clc; clear all; 
close all;

FolderName = '0014';
load(sprintf('true_pose_%s.mat', FolderName))

@initcvekf;
tracker = trackerJPDA('TrackLogic','Integrated' ,'AssignmentThreshold',100,...
    'ConfirmationThreshold', 0.9, ...
    'DeletionThreshold', 0.85);

% Number of frames
files = dir(sprintf('../data_set/%s/oxts/data/*.txt',FolderName));
N = 314;

PtCluster = cell(N,1);
Ptcentroid = cell(N,1);
lidar_vehicle = cell(N,1);
% centroid_world = cell(N,1);
% First Iteration

groundfilename = sprintf('pcd_data/%s/%010dground.pcd', FolderName, 0);
notgroundfilename = sprintf('pcd_data/%s/%010dnotground.pcd', FolderName, 0);
[mapCluster, centroid] = plot_cars_no_ground_aug(groundfilename, notgroundfilename,pose{1});
PtCluster{1} = mapCluster;
Ptcentroid{1} = centroid;
lidar_vehicle{1} = pose{1};

for i = 1:size(Ptcentroid{1},1)
    detection(i) = objectDetection(1,Ptcentroid{1}(i,:));
end

if size(Ptcentroid{1},1) ~= 0
    [confirmed,tentative,alltracks,info] = tracker(detection,1);
end

for i = 1:N
    groundfilename = sprintf('pcd_data/%s/%010dground.pcd', FolderName, i-1);
    notgroundfilename = sprintf('pcd_data/%s/%010dnotground.pcd', FolderName, i-1);
    [mapCluster, centroid] = plot_cars_no_ground_aug(groundfilename, notgroundfilename,pose{i});
    
    hold on
    path_x = pose{i}(1,4);
    path_y = pose{i}(2,4);
    path_z = pose{i}(3,4);
    xlabel('x')
    ylabel('y')
    zlabel('z')
    hold on
    scatter3(path_x,path_y,path_z,20,'red')
    
%     centroid_world{i} = [centroid, ones(size(centroid,1),1)].';
%     centroid_world{i} = pose{i} * centroid_temp;
    PtCluster{i} = mapCluster;
    Ptcentroid{i} = centroid;
    lidar_vehicle{i} = pose{i};
    num_vehicles = JPDA_func(i, tracker,confirmed,tentative,alltracks, Ptcentroid{i});
    
end