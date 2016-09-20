function out = parking_point(vertex, param, x, y, angle)

x1 = param(1);
y1 = param(2);

angle_1 = angle*pi/180;
%%
angle = atan2(y,x);
theta = atan2((vertex(4) - vertex(2)) , (vertex(3) - vertex(1)));
theta2 = theta + angle;
length = sqrt(x^2 + y^2);

x2 = vertex(3) + length*cos(theta2);
y2 = vertex(4) + length*sin(theta2);

out = [x2 y2 theta+angle_1];