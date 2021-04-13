% Please put this .m file inside velodyne_points but outside the data
% folder. This file will read all the .bin files inside the data folder and
% translate them in to rows of [x,y,z,intensity]. Then it will save the
% Point cloud as .mat file
clc; clear all; close all

files = dir('data/*.bin');
N = length(files);
Points_ = cell(N,1);
% Each cell will contain all the points at that timestamp;
for i = 1:N
    fileID = fopen(sprintf('data/%010d.bin',i-1),'rb');
    velo = fread(fileID,[4 inf],'single')';
    Points_{i} = velo;
end
% Save the file as .mat format
save('../matlab/fulldata.mat', 'Points_');