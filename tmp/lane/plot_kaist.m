clear all; close all;

load('tm_KAIST.mat');
load('laser_KAIST.mat');
a = imread('aerial_image_KAIST.png');

%kaist
x_tm_origin = 23193584;
y_tm_origin = 31960261;
u = (acc_xtm + 80 - x_tm_origin)*4;
v = (acc_ytm - 80 - y_tm_origin)*4;

figure;
imshow(a);
hold on;
plot(lane(:,1)*4+2805.5,-lane(:,2)*4+167,'b.')
plot(u, v, '.r', 'MarkerSize', 5);








