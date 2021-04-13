function [mapCluster, centroid] = plot_cars_no_ground_aug(groundfilename, notgroundfilename,pose)
    ptCloud_ground = pcread(groundfilename);
    ptCloud_notground = pcread(notgroundfilename);

    rng('default')
    xy_ground = double(ptCloud_ground.Location(:,1:3));
    xy_notground = double(ptCloud_notground.Location(:,1:3));
    size(ptCloud_ground.Location);

    %% delete outliers:
%     dist_to_lidar = sqrt(xy_ground(:,1).^2 + xy_ground(:,2).^2);
%     [~,idx,~] = deleteoutliers(dist_to_lidar, 0.05,1);
%     x_ground = true(size(xy_ground(:,1)));
%     y_ground = true(size(xy_ground(:,2)));
%     x_ground(idx) = false;
%     y_ground(idx) = false;

%     x = xy_ground(x_ground,1);
%     y = xy_ground(y_ground,2);
%     k = boundary(x,y);

    x = xy_ground(:, 1);
    y = xy_ground(:, 2);
    k = convhull(x,y);

    xv = x(k); yv = y(k);
    % things on the ground:
    xq = xy_notground(:,1); yq = xy_notground(:,2);
    in = inpolygon(xq,yq,xv,yv);

    %% clustering:
    ptCloudWithoutGround = select(ptCloud_notground,in);
    minDistance = 1;
    [labels,numClusters] = pcsegdist(ptCloudWithoutGround,minDistance);

    idxValidPoints = find(labels);
    % label color index contains the labels of the valid points
    labelColorIndex = labels(idxValidPoints);
    segmentedPtCloud = select(ptCloudWithoutGround,idxValidPoints);

    %% Put the points into its cluster:
    mapCluster = cell(numClusters,1);
    for i = 1:length(labelColorIndex)
        current_label = labelColorIndex(i);
        point = segmentedPtCloud.Location(i,:);
        mapCluster{current_label}(end + 1, :) = point;
    end
%     filter
    trueCluster = cell(1,1);
    idx = 1;
    for current_label = 1:numClusters
        current_obj =  mapCluster{current_label};
        current_obj_x = current_obj(:,1);
        current_obj_y = current_obj(:,2);
        current_obj_z = current_obj(:,3);
        obj_length = max(current_obj_x) - min(current_obj_x);
        obj_width = max(current_obj_y) - min(current_obj_y);
        obj_height = max(current_obj_z) - min(current_obj_z);

        % exclude the following clusters:
        size_limit = size(current_obj,1) < 100 || size(current_obj,1) > 600;
        length_limit = obj_length > 6;
        width_limit = obj_width > 5;
        pole_limit = obj_width < 1.5 && obj_width < 1.5 && obj_height > 1.8;
        % 4.27m is the height of a truck
        height_limit = obj_height > 4.27 || obj_height < 0.5;
        % 1.5m * 1.5m = 2.25m^2
%         area_limit = obj_width * obj_length < 2.25;
        
        if size_limit || length_limit || width_limit || height_limit || pole_limit
            continue
        else
            % Volume of box:
            [box, volbox_Rec] = minRectangle(double(current_obj));
            centroid_temp = mean(box);
            dist_temp_x = abs(centroid_temp(1));
            dist_temp_y = centroid_temp(2);
            dist_limit = dist_temp_x > 30 || dist_temp_y > 12 ||  dist_temp_y < 3;
            box_limit = volbox_Rec < 4; % [m^2]
            if box_limit || dist_limit
                continue
            end
        end
        trueCluster{idx,1} = current_obj;
        idx = idx + 1;
    end
 %%
    figure(100)
    idx = 1;
    centroid = zeros(length(trueCluster),3);
    for current_label = 1:length(trueCluster)      
        current_obj= trueCluster{current_label};
        if size(current_obj,2) < 3
            continue
        end
        current_obj_temp = [current_obj, ones(size(current_obj,1),1)].';
        current_obj_world = pose * current_obj_temp;
        
        current_obj_x = current_obj_world(1,:);
        current_obj_y = current_obj_world(2,:);
        current_obj_z = current_obj_world(3,:);   
        
        hold on
        plot3(current_obj_x, current_obj_y, current_obj_z,'.')
        hold on
        % Box fitting on the clusters:
        [box, ~] = minRectangle(double(current_obj_world(1:3,:).'));
        minboxplot(box)   
        centroid(idx,1:3) = mean(box);
        scatter3(centroid(idx,1), centroid(idx,2), centroid(idx,3), 15)
        idx = idx + 1;
        axis equal
        grid on
    end
end