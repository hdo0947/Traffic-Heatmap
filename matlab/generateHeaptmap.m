load("l2_intensity.mat");
load("l1_intensity.mat");
load("oxts.mat")
N = size(oxts_, 1);
intensity_points = zeros(N, 4);
for i=1:N
    intensity_points(i, 1:2) = oxts_{i}(1:2);
    intensity_points(i, 3) = l2_intensity(i, 1);
    intensity_points(i, 4) = l1_intensity(i, 1);
end
save("../representation/intensity_points.mat", 'intensity_points')