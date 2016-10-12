m = 1300;
d = 0.5*1.293*0.35*2.25072;
v = [0:0.1:160]/3.6;
g = 9.81;
v0 = 8/3.6;
theta =0/180*pi;

a0 = -d/m*30*v0-d/m*v0.^2;
a = -d/m*30*v-d/m*v.^2-g*sin(theta)-a0;


F0 = (30*d*v0+d*v0.^2);
ad = -0.1*9.81;
F = (ad*m+30*d*v+m*g*sin(theta)+d*v.^2)-F0;
FF = (abs(F/m)>0.01).*F/m;
figure()
plot(v*3.6,FF*0.5)
% figure()
% plot(v*3.6,a)