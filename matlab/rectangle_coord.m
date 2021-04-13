function coord = rectangle_coord(vector)
    vector = cell2mat(vector);
    x_min = vector(1);
    x_max = vector(3);
    y_min = vector(2);
    y_max = vector(4);
    
    coord = [x_min, y_min; x_min,y_max; x_max,y_max; x_max, y_min;x_min, y_min ]';
end