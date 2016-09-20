clear all; close all;

load('tm_COEX.mat');
load('laser_COEX.mat');
a = imread('aerial_image_COEX.png');

%coex
x_tm_origin = 20450304;
y_tm_origin = 44868269;
u = (acc_xtm + 80 - x_tm_origin)*4;
v = (acc_ytm - 80 - y_tm_origin)*4;

figure;
imshow(a);
hold on;
% //plot(lane(:,1)*4+3615.6,-lane(:,2)*4+7910,'b.')
plot(lane(:,1)*4+3615.6,-lane(:,2)*4+7889,'b.')
plot(u, v, '.r', 'MarkerSize', 5);


figure()
hold on
plot(lane(:,1),lane(:,2),'b.')
plot((u-3615.6)./4, (7889-v)./4, '.r', 'MarkerSize', 5);
axis equal


xxx = (u-3615.6)./4;
yyy = (7889-v)./4






