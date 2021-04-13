clc; clear; close all;

load("cars1_5.mat");

%% Plot car 1 to car 5:
figure(21)
scatter3(car1(:,1),car1(:,2),car1(:,3),2,'filled')
xlabel('x')
ylabel('y')
zlabel('z')
axis equal

figure(22)
scatter3(car2(:,1),car2(:,2),car2(:,3),2,'filled')
xlabel('x')
ylabel('y')
zlabel('z')
axis equal

figure(23)
scatter3(car3(:,1),car3(:,2),car3(:,3),2,'filled')
xlabel('x')
ylabel('y')
zlabel('z')
axis equal

figure(24)
scatter3(car4(:,1),car4(:,2),car4(:,3),2,'filled')
xlabel('x')
ylabel('y')
zlabel('z')
axis equal

figure(25)
scatter3(car5(:,1),car5(:,2),car5(:,3),2,'filled')
xlabel('x')
ylabel('y')
zlabel('z')
axis equal