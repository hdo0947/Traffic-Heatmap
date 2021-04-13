% This file will generate plot of point clouds. First we need to clear the
% workspace and read the data in fulldata.mat
clc; clear all; close all;

load('fulldata.mat')
load('true_pose.mat')

%% Plot full pcd:
figure(31)
skip_ = 1;
tot_index = length(Points_);
% Plot timestamp 0 to timestamp (user defined value - 1):
for i = 1:1
    X = Points_{i}(1:skip_:end,1:3);
    X_1 = [X, ones(size(X,1),1)].';
    X_trans = pose{i} * X_1;
%     x = Points_{i}(1:skip_:end,1);
%     y = Points_{i}(1:skip_:end,2);
%     z = Points_{i}(1:skip_:end,3);
    x = X_trans(1,:);
    y = X_trans(2,:);
    z = X_trans(3,:);
    
    path_x = pose{i}(1,4);
    path_y = pose{i}(2,4);
    path_z = pose{i}(3,4);
    
    intensity = Points_{i}(1:skip_:end,4);

    pct = prctile(intensity,75);
    intensity(intensity > pct ) = pct;
    
    
%     scatter3(x(z > path_z - 1.5),y(z > path_z - 1.5),z(z > path_z - 1.5),2,intensity(z > path_z - 1.5),'filled')
    scatter3(x,y,z,2,intensity,'filled')
    xlabel('x')
    ylabel('y')
    zlabel('z')
    hold on
    scatter3(path_x,path_y,path_z,20,'red')
    % axis equal
end
axis equal
