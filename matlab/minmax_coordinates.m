function [x_min, y_min, x_max, y_max] = minmax_coordinates(points)

x_min = min(points(:,1));
x_max = max(points(:,1));
y_min = min(points(:,2));
y_max = max(points(:,2));

end