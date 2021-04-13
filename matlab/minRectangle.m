function [minRec, volbox_Rec] = minRectangle(car)

[x_min, y_min, x_max, y_max] = minmax_coordinates(car);
coord = [x_min, y_min; x_min,y_max; x_max,y_max; x_max, y_min;x_min, y_min ]';
z_min = min(car(:,3));
z_max = max(car(:,3));
k = convhull(car(:,1), car(:,2));

edge_coordinates = [car(k,1), car(k,2)];

n = length(edge_coordinates);

edges = edge_coordinates(2:end,:) - edge_coordinates(1:n-1,:);

angles = atan2(edges(:,2), edges(:,1));

new_edge_coordinates = zeros(2,length(k));
rect_areas = zeros(length(n)-1,1);
minmax_array = {};
%rect_areas(n,1) = abs((x_max - x_min)*(y_max - y_min));
for j = 1:length(angles)
    rotation_mat = [cos(angles(j)), -sin(angles(j)); sin(angles(j)), cos(angles(j))];
    for i = 1:length(edge_coordinates)
        new_edge_coordinates(:,i) = rotation_mat*edge_coordinates(i,:)';
    end
    [x_min, y_min, x_max, y_max] = minmax_coordinates(new_edge_coordinates');
    minmax_array{j} = [x_min, y_min, x_max, y_max];
    rect_areas(j,1) = abs((x_max - x_min)*(y_max - y_min)); 
end
[v,i] = min(rect_areas);
volbox_Rec = abs(rect_areas(i)) * abs(z_max - z_min);
new_coord = rectangle_coord(minmax_array(i));
rotation_mat = [cos(angles(i)), -sin(angles(i)); sin(angles(i)), cos(angles(i))];
trans_coord = zeros(2,5);
for i = 1:5
    trans_coord(:,i) = new_coord(:,i)'*rotation_mat;
end

coord1 = [trans_coord(1,1) trans_coord(2,1) z_min];
coord2 = [trans_coord(1,2) trans_coord(2,2) z_min];
coord3 = [trans_coord(1,3) trans_coord(2,3) z_min];
coord4 = [trans_coord(1,4) trans_coord(2,4) z_min];
coord5 = [trans_coord(1,1) trans_coord(2,1) z_max];
coord6 = [trans_coord(1,2) trans_coord(2,2) z_max];
coord7 = [trans_coord(1,3) trans_coord(2,3) z_max];
coord8 = [trans_coord(1,4) trans_coord(2,4) z_max];

minRec = [coord1; coord2; coord3; coord4; coord5; coord6; coord7; coord8];
% figure(1)
% hold on
% plot(car(:,1), car(:,2),'b*')
% plot(coord(1,:), coord(2,:),'r')
% plot(car(k,1), car(k,2),'c')
% plot(trans_coord(1,:), trans_coord(2,:),'k')
% 
% figure(2)
% hold on
% plot3(car(:,1), car(:,2), car(:,3),'b*')
%minboxplot(box_coord)
end