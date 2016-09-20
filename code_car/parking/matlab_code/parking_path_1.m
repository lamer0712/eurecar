function out = parking_path_1(park_point, state)

% align final heading
% state and park_point
% r_min = 4;
x_1 = [];
angle = 15*pi/180;

x2 = park_point(1);
y2 = park_point(2);
theta2 = park_point(3);
theta2 = wrap(theta2);

x1 = state(1);
y1 = state(2);
theta1 = state(3);
theta1 = wrap(theta1);

del_theta = theta2 - theta1;

xy = [cos(theta1) sin(theta1) ; -sin(theta1) cos(theta1)] * [x2 - x1 ; y2 - y1];

x = xy(1);
y = xy(2);

A = [x^3 x^2 ; 3*x^2 2*x]^-1 * [y ; tan(del_theta)];

xnew = [0 : 0.1 : x];
ynew = A(1) * xnew.^3 + A(2) * xnew.^2;
xy_1 = [cos(theta1) -sin(theta1) ; sin(theta1) cos(theta1)] * [xnew ; ynew];

xnew = xy_1(1,:) + x1;
ynew = xy_1(2,:) + y1;

% hold on;
% plot(xnew,ynew,'k');
% hold on;

out = [xnew' ynew'];