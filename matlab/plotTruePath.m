clc; clear all; close all;
load('true_pose.mat');
load('fulldata.mat');

%% Plot
skip_ = 1;
tot_index = length(pose);
% Plot timestamp 0 to (user defined value):
for i = 1:tot_index
    x = pose{i}(1,4);
    y = pose{i}(2,4);
    z = pose{i}(3,4);
%     intensity = Points_{i}(1:skip_:end,4);
    % Assign color
%     color_r = 256 * intensity;
%     color_g = 0 * intensity;
%     color_b = 0 * intensity;
%     colors_ = [color_r,color_g,color_b];
    
    figure(1)
    scatter3(x,y,z,2,'filled')
    xlabel('x')
    ylabel('y')
    zlabel('z')
    hold on
    % axis equal
end
hold off
axis equal
