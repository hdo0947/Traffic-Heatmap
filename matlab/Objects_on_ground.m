clc; clear;close all;
ptCloud_ground = pcread('pcd_data/0014/0000000000ground.pcd');
ptCloud_notground = pcread('pcd_data/0014/0000000000notground.pcd');

rng('default')
xy_ground = double(ptCloud_ground.Location(:,1:3));
xy_notground = double(ptCloud_notground.Location(:,1:3));
size(ptCloud_ground.Location);
% plot(xy_ground(:,1), xy_ground(:,2), '.')

figure(1)
plot(xy_ground(:,1), xy_ground(:,2), '.')
k = boundary(xy_ground(:,1), xy_ground(:,2));
hold on;
plot(xy_ground(k,1),xy_ground(k,2))
xlabel('x [m]')
ylabel('y [m]')
title('Perfect Boundary (Ground Points)')
axis equal

% boundary of the ground:
xv = xy_ground(k,1); yv = xy_ground(k,2);
% things on the ground:
xq = xy_notground(:,1); yq = xy_notground(:,2);
in = inpolygon(xq,yq,xv,yv);
figure(2)
plot(xy_ground(k,1),xy_ground(k,2))
hold on
plot(xq,yq,'.')
xlabel('x [m]')
ylabel('y [m]')
title('Perfect Boundary (Non-Ground Points)')
axis equal
%%
% delete outliers:
% dist_to_lidar = sqrt(xy_ground(:,1).^2 + xy_ground(:,2).^2);
% [B,idx,outliers] = deleteoutliers(dist_to_lidar, 0.05,1);
% x_ground = true(size(xy_ground(:,1)));
% y_ground = true(size(xy_ground(:,2)));
% x_ground(idx) = false;
% y_ground(idx) = false;
% 
% x = xy_ground(x_ground,1);
% y = xy_ground(y_ground,2);
x = xy_ground(:,1);
y = xy_ground(:,2);
k = convhull(x,y);
figure(3)
plot(x, y, '.')
hold on;
plot(x(k), y(k))
xlabel('x [m]')
ylabel('y [m]')
title('Convex Boundary (Ground Points)')
axis equal

% boundary of the ground:
xv = x(k); yv = y(k);
% things on the ground:
xq = xy_notground(:,1); yq = xy_notground(:,2);
in = inpolygon(xq,yq,xv,yv);
figure(4)
plot(xv,yv)
hold on
plot(xq,yq,'.')
xlabel('x [m]')
ylabel('y [m]')
title('Convex Boundary (Non-Ground Points)')
axis equal


%% Matlab built in clustering method based on Euclidean distance:
ptCloudWithoutGround = select(ptCloud_notground,in);
minDistance = 1.2;
[labels,numClusters] = pcsegdist(ptCloudWithoutGround,minDistance);

idxValidPoints = find(labels);
% label color index contains the labels of the valid points
labelColorIndex = labels(idxValidPoints);
segmentedPtCloud = select(ptCloudWithoutGround,idxValidPoints);

figure(6)
colormap(hsv(numClusters))
pcshow(segmentedPtCloud.Location,labelColorIndex)
title('Point Cloud Clusters')

%% Put the points into its cluster:
% mapCluster = containers.Map;
% for i = 1:length(labelColorIndex)
%     current_label = string(labelColorIndex(i));
%     point = segmentedPtCloud.Location(i,:);
%     if ~isKey(mapCluster, current_label)
%         mapCluster(current_label) = point;
%     end
%     mapCluster(current_label) = [mapCluster(current_label); point];
% end

mapCluster = cell(numClusters,1);
for i = 1:length(labelColorIndex)
    current_label = string(labelColorIndex(i));
    point = segmentedPtCloud.Location(i,:);
    mapCluster{labelColorIndex(i)}(end + 1, :) = point;
end

trueCluster = cell(1,1);
idx = 1;
for i = 1:numClusters
    current_label = i;
    current_obj =  mapCluster{i};
    current_obj_x = current_obj(:,1);
    current_obj_y = current_obj(:,2);
    current_obj_z = current_obj(:,3);
    obj_length = max(current_obj_x) - min(current_obj_x);
    obj_width = max(current_obj_y) - min(current_obj_y);
    obj_height = max(current_obj_z) - min(current_obj_z);
    
    % exclude the following clusters:
    size_limit = size(mapCluster{current_label},1) < 100 || size(mapCluster{current_label},1) > 600;
    length_limit = obj_length > 6;
    width_limit = obj_width > 5;
    pole_limit = obj_width < 1.5 && obj_width < 1.5 && obj_height > 2;
    % 4.27m is the height of a truck
    height_limit = obj_height > 4.27;
    % 1.5m * 1.5m = 2.25m^2
%     area_limit = obj_width * obj_length < 2.25;
    
    if ~(size_limit || length_limit || width_limit || height_limit || pole_limit)
        trueCluster{idx,1} = mapCluster{i};
        idx = idx + 1;
    end
end

% for i = 1:numClusters
%     current_label = string(i);
%     current_obj =  mapCluster(current_label);
%     current_obj_x = current_obj(:,1);
%     current_obj_y = current_obj(:,2);
%     current_obj_z = current_obj(:,3);
%     obj_length = max(current_obj_x) - min(current_obj_x);
%     obj_width = max(current_obj_y) - min(current_obj_y);
%     obj_height = max(current_obj_z) - min(current_obj_z);
%     
%     % exclude the following clusters:
%     size_limit = size(mapCluster(current_label),1) < 100 || size(mapCluster(current_label),1) > 600;
%     length_limit = obj_length > 6;
%     width_limit = obj_width > 5;
%     pole_limit = obj_width < 1.5 && obj_width < 1.5 && obj_height > 2;
%     % 4.27m is the height of a truck
%     height_limit = obj_height > 4.27;
%     % 1.5m * 1.5m = 2.25m^2
% %     area_limit = obj_width * obj_length < 2.25;
%     
%     if size_limit || length_limit || width_limit || height_limit || pole_limit
%         remove(mapCluster, current_label);
%     end
% end
%% Plot remaining objects:
figure(5)
for i = 1:length(trueCluster)
    current_obj= trueCluster{i};
    current_obj_x = current_obj(:,1);
    current_obj_y = current_obj(:,2);
    current_obj_z = current_obj(:,3);
    plot3(current_obj_x, current_obj_y, current_obj_z,'.')
    hold on
    axis equal
    grid on
end