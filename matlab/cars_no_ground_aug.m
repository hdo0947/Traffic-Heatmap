function [trueCluster_oppodir, trueCluster_samedir, centroid_oppodir, centroid_samedir] = cars_no_ground_aug(groundfilename, notgroundfilename,pose)
    ptCloud_ground = pcread(groundfilename);
    ptCloud_notground = pcread(notgroundfilename);

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
    %% Consider the points above the ground only:
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
    trueCluster_oppodir = cell(1,1);
    trueCluster_samedir = cell(1,1);
    idx_oppodir = 1;
    idx_samedir = 1;
    for current_label = 1:numClusters
        current_obj =  mapCluster{current_label};
        current_obj_x = current_obj(:,1);
        current_obj_y = current_obj(:,2);
        current_obj_z = current_obj(:,3);
        obj_length = max(current_obj_x) - min(current_obj_x);
        obj_width = max(current_obj_y) - min(current_obj_y);
        obj_height = max(current_obj_z) - min(current_obj_z);

        % exclude the following clusters:
        y_location_limit = (max(current_obj_y) > 12 || max(current_obj_y) < -3);
        x_location_limit = (min(current_obj_x) > 10 || max(current_obj_y) < -10);
        size_limit = size(mapCluster{current_label},1) < 80 || (size(current_obj,1) > 1000 && (y_location_limit || x_location_limit));
        length_limit = obj_length > 6;
        width_limit = obj_width > 5;
        pole_limit = (obj_width < 1.5 || obj_length < 1.5) && obj_height > 1.8;
        tree_limit = (obj_width - obj_length) < 0.3 && obj_height > 1.8;
        % 4.27m is the height of a truck
        height_limit = obj_height > 4.27 || obj_height < 0.5;
        % 1.5m * 1.5m = 2.25m^2
%         area_limit = obj_width * obj_length < 2.25;
        
        if size_limit || length_limit || width_limit || height_limit || pole_limit || tree_limit
            continue
        else
            % Volume of box:
            [box, volbox_Rec] = minRectangle(double(current_obj));
            centroid_temp = mean(box);
            dist_temp_x = abs(centroid_temp(1));
            dist_temp_y = centroid_temp(2);
            % limitation for cars running on opposite direction:
            dist_limit_oppodir = dist_temp_x <= 30 && dist_temp_y <= 20 &&  dist_temp_y >= 4;
            % limitation for cars running in the same direction:
            dist_limit_samedir = dist_temp_x <= 30 && dist_temp_y >= -3 && dist_temp_y < 4;
            % Limitation on the size of the car:
            box_limit = volbox_Rec < 3; % [m^2]
            
            if box_limit
                continue
            end
            
            if dist_limit_oppodir
                trueCluster_oppodir{idx_oppodir,1} = current_obj;
                idx_oppodir = idx_oppodir + 1;
            elseif dist_limit_samedir
                trueCluster_samedir{idx_samedir,1} = current_obj;
                idx_samedir = idx_samedir + 1;
            end              
        end
        
    end
 %% Plotting
    idx = 1;
    centroid_oppodir = [];
    centroid_samedir = [];
    
%     figure(100)
    % plot cars on the opposite direction and calculate the centroid:
    for current_label = 1:length(trueCluster_oppodir)      
        current_obj= trueCluster_oppodir{current_label};
        % check if that cell is empty:
        if size(current_obj,2) < 3
            continue
        end
        current_obj_temp = [current_obj, ones(size(current_obj,1),1)].';
        current_obj_world = pose * current_obj_temp;
        
        current_obj_x = current_obj_world(1,:);
        current_obj_y = current_obj_world(2,:);
        current_obj_z = current_obj_world(3,:);   
        
%         hold on
%         plot3(current_obj_x, current_obj_y, current_obj_z,'.')
%         hold on
        % Box fitting on the clusters:
        [box, ~] = minRectangle(double(current_obj_world(1:3,:).'));
%         minboxplot(box)   
        centroid_oppodir(idx,1:3) = mean(box);
%         scatter3(centroid_oppodir(idx,1), centroid_oppodir(idx,2), centroid_oppodir(idx,3), 15)
        idx = idx + 1;
%         axis equal
%         grid on
    end
    
    % plot cars on the same direction and calculate the centroid:
    idx = 1;
    for current_label = 1:length(trueCluster_samedir)      
        current_obj= trueCluster_samedir{current_label};
        % check if that cell is empty:
        if size(current_obj,2) < 3
            continue
        end
        current_obj_temp = [current_obj, ones(size(current_obj,1),1)].';
        current_obj_world = pose * current_obj_temp;
        
        current_obj_x = current_obj_world(1,:);
        current_obj_y = current_obj_world(2,:);
        current_obj_z = current_obj_world(3,:);   
        
%         hold on
%         plot3(current_obj_x, current_obj_y, current_obj_z,'.')
%         hold on
        % Box fitting on the clusters:
        [box, ~] = minRectangle(double(current_obj_world(1:3,:).'));
%         minboxplot(box)   
        centroid_samedir(idx,1:3) = mean(box);
%         scatter3(centroid_samedir(idx,1), centroid_samedir(idx,2), centroid_samedir(idx,3), 15)
        idx = idx + 1;
%         axis equal
%         grid on
    end
end